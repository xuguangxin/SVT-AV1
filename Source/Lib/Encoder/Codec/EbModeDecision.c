/*
* Copyright(c) 2019 Intel Corporation
* Copyright (c) 2016, Alliance for Open Media. All rights reserved
*
* This source code is subject to the terms of the BSD 2 Clause License and
* the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
* was not distributed with this source code in the LICENSE file, you can
* obtain it at https://www.aomedia.org/license/software-license. If the Alliance for Open
* Media Patent License 1.0 was not distributed with this source code in the
* PATENTS file, you can obtain it at https://www.aomedia.org/license/patent-license.
*/

/***************************************
* Includes
***************************************/
#include <stdlib.h>
#include <limits.h>

#include "EbCommonUtils.h"
#include "EbSequenceControlSet.h"
#include "EbModeDecision.h"
#include "EbTransformUnit.h"
#include "EbModeDecisionProcess.h"
#include "EbMotionEstimation.h"

#include "av1me.h"
#include "hash.h"
#include "EbEncInterPrediction.h"
#include "EbRateDistortionCost.h"
#include "aom_dsp_rtcd.h"
#include "EbLog.h"
#include "EbResize.h"
#if UPGRADE_SUBPEL
#include "mcomp.h"
#endif
#define INCRMENT_CAND_TOTAL_COUNT(cnt)                                                     \
    MULTI_LINE_MACRO_BEGIN cnt++;                                                          \
    if (cnt >= MODE_DECISION_CANDIDATE_MAX_COUNT_Y)                                          \
        SVT_LOG(" ERROR: reaching limit for MODE_DECISION_CANDIDATE_MAX_COUNT %i\n", cnt); \
    MULTI_LINE_MACRO_END

int8_t av1_ref_frame_type(const MvReferenceFrame *const rf);
int    av1_filter_intra_allowed_bsize(uint8_t enable_filter_intra, BlockSize bs);

void av1_set_ref_frame(MvReferenceFrame *rf, int8_t ref_frame_type);

#if LOG_MV_VALIDITY
void check_mv_validity(int16_t x_mv, int16_t y_mv, uint8_t need_shift);
#endif
static INLINE int is_interintra_allowed_bsize(const BlockSize bsize) {
    return (bsize >= BLOCK_8X8) && (bsize <= BLOCK_32X32);
}

static INLINE int is_interintra_allowed_mode(const PredictionMode mode) {
    return (mode >= SINGLE_INTER_MODE_START) && (mode < SINGLE_INTER_MODE_END);
}

static INLINE int is_interintra_allowed_ref(const MvReferenceFrame rf[2]) {
    return (rf[0] > INTRA_FRAME) && (rf[1] <= INTRA_FRAME);
}
int svt_is_interintra_allowed(uint8_t enable_inter_intra, BlockSize sb_type, PredictionMode mode,
                              const MvReferenceFrame ref_frame[2]) {
    return enable_inter_intra && is_interintra_allowed_bsize((const BlockSize)sb_type) &&
           is_interintra_allowed_mode(mode) && is_interintra_allowed_ref(ref_frame);
}
//Given one reference frame identified by the pair (list_index,ref_index)
//indicate if ME data is valid
uint8_t is_me_data_present(
    struct ModeDecisionContext  *context_ptr,
    const MeSbResults           *me_results,
    uint8_t                      list_idx,
    uint8_t                      ref_idx){
    uint8_t total_me_cnt = me_results->total_me_candidate_index[context_ptr->me_block_offset];
#if ME_MEM_OPT
    const MeCandidate *me_block_results = &me_results->me_candidate_array[context_ptr->me_cand_offset];
#else
    const MeCandidate *me_block_results = me_results->me_candidate[context_ptr->me_block_offset];
#endif
    for (uint32_t me_cand_i = 0; me_cand_i < total_me_cnt; ++me_cand_i){
        const MeCandidate *me_cand = &me_block_results[me_cand_i];
        assert(me_cand->direction <= 2);
        if (me_cand->direction == 0 || me_cand->direction == 2) {
            if (list_idx == me_cand->ref0_list && ref_idx == me_cand->ref_idx_l0)
                return 1;
        }
        if (me_cand->direction == 1 || me_cand->direction == 2) {
            if (list_idx == me_cand->ref1_list && ref_idx == me_cand->ref_idx_l1)
                return 1;
        }
    }
    return 0;
}
/********************************************
* Constants
********************************************/
// 1 - Regular uni-pred ,
// 2 - Regular uni-pred + Wedge compound Inter Intra
// 3 - Regular uni-pred + Wedge compound Inter Intra + Smooth compound Inter Intra

#define II_COUNT 3

static INLINE int is_inter_mode(PredictionMode mode) {
    return mode >= SINGLE_INTER_MODE_START && mode < SINGLE_INTER_MODE_END;
}
#if INCREASE_WM_CANDS
EbBool warped_motion_mode_allowed(PictureControlSet* pcs, ModeDecisionContext* ctx) {
    FrameHeader *frm_hdr = &pcs->parent_pcs_ptr->frm_hdr;
    return frm_hdr->allow_warped_motion && has_overlappable_candidates(ctx->blk_ptr) &&
        ctx->blk_geom->bwidth >= 8 && ctx->blk_geom->bheight >= 8 &&
        ctx->warped_motion_injection;
}
#endif
MotionMode obmc_motion_mode_allowed(const PictureControlSet *   pcs_ptr,
                                    struct ModeDecisionContext *context_ptr, const BlockSize bsize,
                                    MvReferenceFrame rf0, MvReferenceFrame rf1,
                                    PredictionMode mode) {
#if OBMC_FAST
    if (!context_ptr->obmc_ctrls.enabled) return SIMPLE_TRANSLATION;
#else
    if (!context_ptr->md_pic_obmc_mode) return SIMPLE_TRANSLATION;
#endif
    FrameHeader *frm_hdr = &pcs_ptr->parent_pcs_ptr->frm_hdr;

    if (!frm_hdr->is_motion_mode_switchable) return SIMPLE_TRANSLATION;

    if (frm_hdr->force_integer_mv == 0) {
        const TransformationType gm_type = pcs_ptr->parent_pcs_ptr->global_motion[rf0].wmtype;
        if (is_global_mv_block(mode, bsize, gm_type)) return SIMPLE_TRANSLATION;
    }

    if (is_motion_variation_allowed_bsize(bsize) && is_inter_mode(mode) && rf1 != INTRA_FRAME &&
        !(rf1 > INTRA_FRAME)) // is_motion_variation_allowed_compound
    {
        if (!has_overlappable_candidates(context_ptr->blk_ptr)) // check_num_overlappable_neighbors
            return SIMPLE_TRANSLATION;

        return OBMC_CAUSAL;
    } else
        return SIMPLE_TRANSLATION;
}

void precompute_obmc_data(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr);

//static uint32_t  AntiContouringIntraMode[11] = { EB_INTRA_PLANAR, EB_INTRA_DC, EB_INTRA_HORIZONTAL, EB_INTRA_VERTICAL,
//EB_INTRA_MODE_2, EB_INTRA_MODE_6, EB_INTRA_MODE_14, EB_INTRA_MODE_18, EB_INTRA_MODE_22, EB_INTRA_MODE_30, EB_INTRA_MODE_34 };
int32_t have_newmv_in_inter_mode(PredictionMode mode) {
    return (mode == NEWMV || mode == NEW_NEWMV || mode == NEAREST_NEWMV || mode == NEW_NEARESTMV ||
            mode == NEAR_NEWMV || mode == NEW_NEARMV);
}
const uint32_t parent_index[85] = {
    0,  0,  0,  2,  2,  2,  2,  0,  7,  7,  7,  7,  0,  12, 12, 12, 12, 0,  17, 17, 17, 17,
    0,  0,  23, 23, 23, 23, 0,  28, 28, 28, 28, 0,  33, 33, 33, 33, 0,  38, 38, 38, 38, 0,
    0,  44, 44, 44, 44, 0,  49, 49, 49, 49, 0,  54, 54, 54, 54, 0,  59, 59, 59, 59, 0,  0,
    65, 65, 65, 65, 0,  70, 70, 70, 70, 0,  75, 75, 75, 75, 0,  80, 80, 80, 80};
/*
  NORMAL ORDER
  |-------------------------------------------------------------|
  | ref_idx          0            1           2            3       |
  | List0            LAST        LAST2        LAST3        GOLD    |
  | List1            BWD            ALT2        ALT                 |
  |-------------------------------------------------------------|
*/
#define INVALID_REF 0xF

uint8_t get_list_idx(uint8_t ref_type) {
    if (ref_type == LAST_FRAME || ref_type == LAST2_FRAME || ref_type == LAST3_FRAME ||
        ref_type == GOLDEN_FRAME)
        return 0;
    else if (ref_type == BWDREF_FRAME || ref_type == ALTREF_FRAME || ref_type == ALTREF2_FRAME)
        return 1;
    else
        return (0);
};

uint8_t get_ref_frame_idx(uint8_t ref_type) {
    if (ref_type == LAST_FRAME || ref_type == BWDREF_FRAME)
        return 0;
    else if (ref_type == LAST2_FRAME || ref_type == ALTREF2_FRAME)
        return 1;
    else if (ref_type == LAST3_FRAME || ref_type == ALTREF_FRAME)
        return 2;
    else if (ref_type == GOLDEN_FRAME)
        return 3;
    else
        return (0);
};
MvReferenceFrame svt_get_ref_frame_type(uint8_t list, uint8_t ref_idx) {
    switch (list) {
    case 0:
        return (ref_idx == 0
                    ? LAST_FRAME
                    : ref_idx == 1
                          ? LAST2_FRAME
                          : ref_idx == 2 ? LAST3_FRAME : ref_idx == 3 ? GOLDEN_FRAME : INVALID_REF);
    case 1:
        return (ref_idx == 0
                    ? BWDREF_FRAME
                    : ref_idx == 1 ? ALTREF2_FRAME : ref_idx == 2 ? ALTREF_FRAME : INVALID_REF);
    default: return (INVALID_REF);
    }
};
extern uint32_t stage1ModesArray[];

uint8_t get_max_drl_index(uint8_t refmvCnt, PredictionMode mode);
int32_t eb_av1_mv_bit_cost(const MV *mv, const MV *ref, const int32_t *mvjcost, int32_t *mvcost[2],
                           int32_t weight);
#define MV_COST_WEIGHT 108
#define MAX_INTERINTRA_SB_SQUARE 32 * 32
EbErrorType intra_luma_prediction_for_interintra(ModeDecisionContext *md_context_ptr,
                                                 PictureControlSet *  pcs_ptr,
                                                 InterIntraMode       interintra_mode,
                                                 EbPictureBufferDesc *prediction_ptr);
int64_t     pick_wedge_fixed_sign(ModeDecisionCandidate *candidate_ptr, PictureControlSet *pcs_ptr,
                                  ModeDecisionContext *context_ptr, const BlockSize bsize,
                                  const int16_t *const residual1, const int16_t *const diff10,
                                  const int8_t wedge_sign, int8_t *const best_wedge_index);
void        model_rd_for_sb_with_curvfit(PictureControlSet *  picture_control_set_ptr,
                                         ModeDecisionContext *context_ptr, BlockSize bsize, int bw, int bh,
                                         uint8_t *src_buf, uint32_t src_stride, uint8_t *pred_buf,
                                         uint32_t pred_stride, int plane_from, int plane_to, int mi_row,
                                         int mi_col, int *out_rate_sum, int64_t *out_dist_sum,
                                         int *plane_rate, int64_t *plane_sse, int64_t *plane_dist);

static int64_t pick_interintra_wedge(ModeDecisionCandidate *candidate_ptr,
                                     PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                                     const BlockSize bsize, const uint8_t *const p0,
                                     const uint8_t *const p1, uint8_t *src_buf, uint32_t src_stride,
                                     int32_t *wedge_index_out) {
    assert(is_interintra_wedge_used(bsize));
    // assert(cpi->common.seq_params.enable_interintra_compound);

    const int bw = block_size_wide[bsize];
    const int bh = block_size_high[bsize];
    DECLARE_ALIGNED(32, int16_t, residual1[MAX_SB_SQUARE]); // src - pred1
    DECLARE_ALIGNED(32, int16_t, diff10[MAX_SB_SQUARE]); // pred1 - pred0
    if (context_ptr->hbd_mode_decision) {
        aom_highbd_subtract_block(bh, bw, residual1, bw, src_buf, src_stride, p1, bw, EB_10BIT);
        aom_highbd_subtract_block(bh, bw, diff10, bw, p1, bw, p0, bw, EB_10BIT);

    } else {
        eb_aom_subtract_block(bh, bw, residual1, bw, src_buf, src_stride, p1, bw);
        eb_aom_subtract_block(bh, bw, diff10, bw, p1, bw, p0, bw);
    }

    int8_t  wedge_index = -1;
    int64_t rd          = pick_wedge_fixed_sign(
        candidate_ptr, pcs_ptr, context_ptr, bsize, residual1, diff10, 0, &wedge_index);

    *wedge_index_out = wedge_index;

    return rd;
}
//for every CU, perform DC/V/H/S intra prediction to be used later in inter-intra search
void precompute_intra_pred_for_inter_intra(PictureControlSet *  pcs_ptr,
                                           ModeDecisionContext *context_ptr) {
    uint32_t            j;
    EbPictureBufferDesc pred_desc;
    pred_desc.origin_x = pred_desc.origin_y = 0;
    pred_desc.stride_y                      = context_ptr->blk_geom->bwidth;

    for (j = 0; j < INTERINTRA_MODES; ++j) {
        InterIntraMode interintra_mode = (InterIntraMode)j;
        pred_desc.buffer_y             = context_ptr->intrapred_buf[j];
        intra_luma_prediction_for_interintra(context_ptr, pcs_ptr, interintra_mode, &pred_desc);
    }
}

void combine_interintra(InterIntraMode mode, int8_t use_wedge_interintra, int wedge_index,
                        int wedge_sign, BlockSize bsize, BlockSize plane_bsize, uint8_t *comppred,
                        int compstride, const uint8_t *interpred, int interstride,
                        const uint8_t *intrapred, int intrastride);
void inter_intra_search(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                        ModeDecisionCandidate *candidate_ptr) {
    DECLARE_ALIGNED(16, uint8_t, tmp_buf[2 * MAX_INTERINTRA_SB_SQUARE]);
    DECLARE_ALIGNED(16, uint8_t, ii_pred_buf[2 * MAX_INTERINTRA_SB_SQUARE]);
    //get inter pred for ref0
    EbPictureBufferDesc *src_pic = context_ptr->hbd_mode_decision
                                       ? pcs_ptr->input_frame16bit
                                       : pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr;
    uint16_t *src_buf_hbd = (uint16_t *)src_pic->buffer_y +
                            (context_ptr->blk_origin_x + src_pic->origin_x) +
                            (context_ptr->blk_origin_y + src_pic->origin_y) * src_pic->stride_y;
    uint8_t *src_buf = src_pic->buffer_y + (context_ptr->blk_origin_x + src_pic->origin_x) +
                       (context_ptr->blk_origin_y + src_pic->origin_y) * src_pic->stride_y;

    uint8_t bit_depth = context_ptr->hbd_mode_decision ? EB_10BIT : EB_8BIT;
    uint32_t full_lambda =  context_ptr->hbd_mode_decision ?
        context_ptr->full_lambda_md[EB_10_BIT_MD] :
        context_ptr->full_lambda_md[EB_8_BIT_MD];

    uint32_t            bwidth  = context_ptr->blk_geom->bwidth;
    uint32_t            bheight = context_ptr->blk_geom->bheight;
    EbPictureBufferDesc pred_desc;
    pred_desc.origin_x = pred_desc.origin_y = 0;
    pred_desc.stride_y                      = bwidth;

    EbPictureBufferDesc *ref_pic_list0;
    EbPictureBufferDesc *ref_pic_list1 = NULL;
    Mv                   mv_0;
    Mv                   mv_1;
    mv_0.x = candidate_ptr->motion_vector_xl0;
    mv_0.y = candidate_ptr->motion_vector_yl0;
    mv_1.x = candidate_ptr->motion_vector_xl1;
    mv_1.y = candidate_ptr->motion_vector_yl1;
    MvUnit mv_unit;
    mv_unit.mv[0]               = mv_0;
    mv_unit.mv[1]               = mv_1;
    int8_t           ref_idx_l0 = candidate_ptr->ref_frame_index_l0;
    int8_t           ref_idx_l1 = candidate_ptr->ref_frame_index_l1;
    MvReferenceFrame rf[2];
    av1_set_ref_frame(rf, candidate_ptr->ref_frame_type);
    uint8_t list_idx0, list_idx1;
    list_idx0 = get_list_idx(rf[0]);
    if (rf[1] == NONE_FRAME)
        list_idx1 = get_list_idx(rf[0]);
    else
        list_idx1 = get_list_idx(rf[1]);
    assert(list_idx0 < MAX_NUM_OF_REF_PIC_LIST);
    assert(list_idx1 < MAX_NUM_OF_REF_PIC_LIST);
    //
    if (ref_idx_l0 >= 0)
        ref_pic_list0 =
            context_ptr->hbd_mode_decision
                ? ((EbReferenceObject *)pcs_ptr->ref_pic_ptr_array[list_idx0][ref_idx_l0]
                       ->object_ptr)
                      ->reference_picture16bit
                : ((EbReferenceObject *)pcs_ptr->ref_pic_ptr_array[list_idx0][ref_idx_l0]
                       ->object_ptr)
                      ->reference_picture;
    else
        ref_pic_list0 = (EbPictureBufferDesc *)NULL;

    if (ref_idx_l1 >= 0)
        ref_pic_list1 =
            context_ptr->hbd_mode_decision
                ? ((EbReferenceObject *)pcs_ptr->ref_pic_ptr_array[list_idx1][ref_idx_l1]
                       ->object_ptr)
                      ->reference_picture16bit
                : ((EbReferenceObject *)pcs_ptr->ref_pic_ptr_array[list_idx1][ref_idx_l1]
                       ->object_ptr)
                      ->reference_picture;
    else
        ref_pic_list1 = (EbPictureBufferDesc *)NULL;

    // Use scaled references if resolution of the reference is different from that of the input
    if(ref_pic_list0 != NULL)
        use_scaled_rec_refs_if_needed(pcs_ptr,
                                      pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr,
                                      (EbReferenceObject *)pcs_ptr->ref_pic_ptr_array[list_idx0][list_idx0]->object_ptr,
                                      &ref_pic_list0);
    if(ref_pic_list1 != NULL)
        use_scaled_rec_refs_if_needed(pcs_ptr,
                                      pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr,
                                      (EbReferenceObject *)pcs_ptr->ref_pic_ptr_array[list_idx1][ref_idx_l1]->object_ptr,
                                      &ref_pic_list1);

    mv_unit.pred_direction = candidate_ptr->prediction_direction[0];

    pred_desc.buffer_y = tmp_buf;

    //we call the regular inter prediction path here(no compound)
    av1_inter_prediction(
        pcs_ptr,
        0, //ASSUMPTION: fixed interpolation filter.
        context_ptr->blk_ptr,
        candidate_ptr->ref_frame_type,
        &mv_unit,
        0, //use_intrabc,
        SIMPLE_TRANSLATION,
        0,
        0,
        1, //compound_idx not used
        NULL, // interinter_comp not used
        NULL,
        NULL,
        NULL,
        NULL,
        0,
        0,
        0,
        0,
        context_ptr->blk_origin_x,
        context_ptr->blk_origin_y,
        bwidth,
        bheight,
        ref_pic_list0,
        ref_pic_list1,
        &pred_desc, //output
        0, //output origin_x,
        0, //output origin_y,
        0, //do chroma
        context_ptr->hbd_mode_decision ? EB_10BIT : EB_8BIT);

    assert(is_interintra_wedge_used(
        context_ptr->blk_geom->bsize)); //if not I need to add nowedge path!!

    int64_t        best_interintra_rd = INT64_MAX;
    int            rate_sum;
    int64_t        dist_sum;
    int            tmp_rate_mv              = 0;
    InterIntraMode best_interintra_mode     = INTERINTRA_MODES;
    for (int j = 0; j < INTERINTRA_MODES; ++j) {
        //if ((!cpi->oxcf.enable_smooth_intra || cpi->sf.disable_smooth_intra) &&
        //    (InterIntraMode)j == II_SMOOTH_PRED)
        //  continue;
        InterIntraMode interintra_mode = (InterIntraMode)j;
        //rmode = interintra_mode_cost[mbmi->interintra_mode];
        const int bsize_group = size_group_lookup[context_ptr->blk_geom->bsize];
        const int rmode       = candidate_ptr->md_rate_estimation_ptr
                              ->inter_intra_mode_fac_bits[bsize_group][interintra_mode];
        //av1_combine_interintra(xd, bsize, 0, tmp_buf, bw, intrapred, bw);
        if (context_ptr->hbd_mode_decision)
            combine_interintra_highbd(interintra_mode, //mode,
                                      0, //use_wedge_interintra,
                                      0, //candidate_ptr->interintra_wedge_index,
                                      0, //int wedge_sign,
                                      context_ptr->blk_geom->bsize,
                                      context_ptr->blk_geom->bsize, // plane_bsize,
                                      ii_pred_buf,
                                      bwidth, /*uint8_t *comppred, int compstride,*/
                                      tmp_buf,
                                      bwidth, /*const uint8_t *interpred, int interstride,*/
                                      context_ptr->intrapred_buf[j],
                                      bwidth /*const uint8_t *intrapred,   int intrastride*/,
                                      bit_depth);
        else

            combine_interintra(interintra_mode, //mode,
                               0, //use_wedge_interintra,
                               0, //candidate_ptr->interintra_wedge_index,
                               0, //int wedge_sign,
                               context_ptr->blk_geom->bsize,
                               context_ptr->blk_geom->bsize, // plane_bsize,
                               ii_pred_buf,
                               bwidth, /*uint8_t *comppred, int compstride,*/
                               tmp_buf,
                               bwidth, /*const uint8_t *interpred, int interstride,*/
                               context_ptr->intrapred_buf[j],
                               bwidth /*const uint8_t *intrapred,   int intrastride*/);

        //model_rd_sb_fn[MODELRD_TYPE_INTERINTRA](
        //    cpi, bsize, x, xd, 0, 0, mi_row, mi_col, &rate_sum, &dist_sum,
        //    &tmp_skip_txfm_sb, &tmp_skip_sse_sb, NULL, NULL, NULL);
        model_rd_for_sb_with_curvfit(
            pcs_ptr,
            context_ptr,
            context_ptr->blk_geom->bsize,
            bwidth,
            bheight,
            context_ptr->hbd_mode_decision ? (uint8_t *)src_buf_hbd : src_buf,
            src_pic->stride_y,
            ii_pred_buf,
            bwidth,
            0,
            0,
            0,
            0,
            &rate_sum,
            &dist_sum,
            NULL,
            NULL,
            NULL);
        // rd = RDCOST(x->rdmult, tmp_rate_mv + rate_sum + rmode, dist_sum);
        int64_t rd = RDCOST(full_lambda, tmp_rate_mv + rate_sum + rmode, dist_sum);

        if (rd < best_interintra_rd) {
            best_interintra_rd             = rd;
            candidate_ptr->interintra_mode = best_interintra_mode = interintra_mode;
        }
    }

    /* best_interintra_rd_wedge =
            pick_interintra_wedge(cpi, x, bsize, intrapred_, tmp_buf_);*/

    //CHKN need to re-do intra pred using the winner, or have a separate intra serch for wedge

    pick_interintra_wedge(candidate_ptr,
                          pcs_ptr,
                          context_ptr,
                          context_ptr->blk_geom->bsize,
                          context_ptr->intrapred_buf[best_interintra_mode],
                          tmp_buf,
                          context_ptr->hbd_mode_decision ? (uint8_t *)src_buf_hbd : src_buf,
                          src_pic->stride_y,
                          &candidate_ptr->interintra_wedge_index);

    //if (best_interintra_rd_wedge < best_interintra_rd) {

    //candidate_ptr->use_wedge_interintra = 1;
    //candidate_ptr->ii_wedge_sign = 0;
    //}
    //args->inter_intra_mode[mbmi->ref_frame[0]] = best_interintra_mode;
    // Enable wedge search if source variance and edge strength are above the thresholds.
}

COMPOUND_TYPE to_av1_compound_lut[] = {
    COMPOUND_AVERAGE, COMPOUND_DISTWTD, COMPOUND_DIFFWTD, COMPOUND_WEDGE};

void determine_compound_mode(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                             ModeDecisionCandidate *candidatePtr, MD_COMP_TYPE cur_type) {
    candidatePtr->interinter_comp.type = to_av1_compound_lut[cur_type];

    if (cur_type == MD_COMP_AVG) {
        candidatePtr->comp_group_idx = 0;
        candidatePtr->compound_idx   = 1;
    } else if (cur_type == MD_COMP_DIST) {
        candidatePtr->comp_group_idx = 0;
        candidatePtr->compound_idx   = 0;
    } else if (cur_type == MD_COMP_DIFF0) {
        candidatePtr->comp_group_idx            = 1;
        candidatePtr->compound_idx              = 1;
        candidatePtr->interinter_comp.mask_type = 55;
        search_compound_diff_wedge(pcs_ptr, context_ptr, candidatePtr);

    }
    //else if (cur_type == MD_COMP_DIFF1) {
    //    candidatePtr->comp_group_idx = 1;
    //    candidatePtr->compound_idx = 1;
    //    candidatePtr->interinter_comp.mask_type = 1;
    //}
    else if (cur_type == MD_COMP_WEDGE) {
        candidatePtr->comp_group_idx = 1;
        candidatePtr->compound_idx   = 1;
        search_compound_diff_wedge(pcs_ptr, context_ptr, candidatePtr);
    } else {
        SVT_LOG("ERROR: not used comp type\n");
    }
}

void choose_best_av1_mv_pred(ModeDecisionContext *           context_ptr,
                             struct MdRateEstimationContext *md_rate_estimation_ptr,
                             BlkStruct *blk_ptr, MvReferenceFrame ref_frame, uint8_t is_compound,
                             PredictionMode mode, //NEW or NEW_NEW
                             int16_t mv0x, int16_t mv0y, int16_t mv1x, int16_t mv1y,
                             uint8_t *bestDrlIndex, // output
                             IntMv    best_pred_mv[2] // output
) {
#if SHUT_FAST_RATE_PD0
    if (context_ptr->shut_fast_rate) {
        return;
    }
#endif
    uint8_t  drli, max_drl_index;
    IntMv    nearestmv[2];
    IntMv    nearmv[2];
    IntMv    ref_mv[2];
    uint32_t best_mv_cost = 0xFFFFFFFF;
    MV       mv;

    max_drl_index = get_max_drl_index(blk_ptr->av1xd->ref_mv_count[ref_frame], mode);
    // max_drl_index = 1;

    for (drli = 0; drli < max_drl_index; drli++) {
        get_av1_mv_pred_drl(
            context_ptr, blk_ptr, ref_frame, is_compound, mode, drli, nearestmv, nearmv, ref_mv);

        //compute the rate for this drli Cand
        mv.row = mv0y;
        mv.col = mv0x;

        uint32_t mv_rate = (uint32_t)eb_av1_mv_bit_cost(&mv,
                                                        &(ref_mv[0].as_mv),
                                                        md_rate_estimation_ptr->nmv_vec_cost,
                                                        md_rate_estimation_ptr->nmvcoststack,
                                                        MV_COST_WEIGHT);

        if (is_compound) {
            mv.row = mv1y;
            mv.col = mv1x;

            mv_rate += (uint32_t)eb_av1_mv_bit_cost(&mv,
                                                    &(ref_mv[1].as_mv),
                                                    md_rate_estimation_ptr->nmv_vec_cost,
                                                    md_rate_estimation_ptr->nmvcoststack,
                                                    MV_COST_WEIGHT);
        }

        if (mv_rate < best_mv_cost) {
            best_mv_cost    = mv_rate;
            *bestDrlIndex   = drli;
            best_pred_mv[0] = ref_mv[0];
            best_pred_mv[1] = ref_mv[1];
        }
    }
}

static void mode_decision_candidate_buffer_dctor(EbPtr p) {
    ModeDecisionCandidateBuffer *obj = (ModeDecisionCandidateBuffer *)p;
    EB_DELETE(obj->prediction_ptr);
#if !CAND_MEM_OPT
    EB_DELETE(obj->prediction_ptr_temp);
    EB_DELETE(obj->cfl_temp_prediction_ptr);
#endif
#if !MEM_OPT_MD_BUF_DESC
    EB_DELETE(obj->residual_ptr);
#endif
#if !CAND_MEM_OPT
    EB_DELETE(obj->residual_quant_coeff_ptr);
#endif
    EB_DELETE(obj->recon_coeff_ptr);
#if !MEM_OPT_MD_BUF_DESC
    EB_DELETE(obj->recon_ptr);
#endif
}
static void mode_decision_scratch_candidate_buffer_dctor(EbPtr p) {
    ModeDecisionCandidateBuffer *obj = (ModeDecisionCandidateBuffer *)p;
    EB_DELETE(obj->prediction_ptr);
#if !CAND_MEM_OPT
    EB_DELETE(obj->prediction_ptr_temp);
    EB_DELETE(obj->cfl_temp_prediction_ptr);
#endif
    EB_DELETE(obj->residual_ptr);
#if ! CAND_MEM_OPT
    EB_DELETE(obj->residual_quant_coeff_ptr);
#endif
    EB_DELETE(obj->recon_coeff_ptr);
    EB_DELETE(obj->recon_ptr);
}
/***************************************
* Mode Decision Candidate Ctor
***************************************/
EbErrorType mode_decision_candidate_buffer_ctor(ModeDecisionCandidateBuffer *buffer_ptr,
                                                EbBitDepthEnum               max_bitdepth,
#if SB64_MEM_OPT
                                                uint8_t sb_size,
#endif
#if MEM_OPT_UV_MODE
                                                uint32_t buffer_desc_mask,
#endif
#if MEM_OPT_MD_BUF_DESC
                                                EbPictureBufferDesc *temp_residual_ptr,
                                                EbPictureBufferDesc *temp_recon_ptr,
#endif
                                                uint64_t *fast_cost_ptr, uint64_t *full_cost_ptr,
                                                uint64_t *full_cost_skip_ptr,
                                                uint64_t *full_cost_merge_ptr) {
    EbPictureBufferDescInitData picture_buffer_desc_init_data;
#if !MEM_OPT_MD_BUF_DESC
    EbPictureBufferDescInitData double_width_picture_buffer_desc_init_data;
#endif

    EbPictureBufferDescInitData thirty_two_width_picture_buffer_desc_init_data;

    buffer_ptr->dctor = mode_decision_candidate_buffer_dctor;

    // Init Picture Data
#if SB64_MEM_OPT
    picture_buffer_desc_init_data.max_width                       = sb_size;
    picture_buffer_desc_init_data.max_height                      = sb_size;
#else
    picture_buffer_desc_init_data.max_width                       = MAX_SB_SIZE;
    picture_buffer_desc_init_data.max_height                      = MAX_SB_SIZE;
#endif
    picture_buffer_desc_init_data.bit_depth                       = max_bitdepth;
    picture_buffer_desc_init_data.color_format                    = EB_YUV420;
#if MEM_OPT_UV_MODE
    picture_buffer_desc_init_data.buffer_enable_mask              = buffer_desc_mask;
#else
    picture_buffer_desc_init_data.buffer_enable_mask              = PICTURE_BUFFER_DESC_FULL_MASK;
#endif
    picture_buffer_desc_init_data.left_padding                    = 0;
    picture_buffer_desc_init_data.right_padding                   = 0;
    picture_buffer_desc_init_data.top_padding                     = 0;
    picture_buffer_desc_init_data.bot_padding                     = 0;
    picture_buffer_desc_init_data.split_mode                      = EB_FALSE;
#if !MEM_OPT_MD_BUF_DESC
#if SB64_MEM_OPT
    double_width_picture_buffer_desc_init_data.max_width          = sb_size;
    double_width_picture_buffer_desc_init_data.max_height         = sb_size;
#else
    double_width_picture_buffer_desc_init_data.max_width          = MAX_SB_SIZE;
    double_width_picture_buffer_desc_init_data.max_height         = MAX_SB_SIZE;
#endif
    double_width_picture_buffer_desc_init_data.bit_depth          = EB_16BIT;
    double_width_picture_buffer_desc_init_data.color_format       = EB_YUV420;
    double_width_picture_buffer_desc_init_data.buffer_enable_mask = PICTURE_BUFFER_DESC_FULL_MASK;
    double_width_picture_buffer_desc_init_data.left_padding       = 0;
    double_width_picture_buffer_desc_init_data.right_padding      = 0;
    double_width_picture_buffer_desc_init_data.top_padding        = 0;
    double_width_picture_buffer_desc_init_data.bot_padding        = 0;
    double_width_picture_buffer_desc_init_data.split_mode         = EB_FALSE;
#endif

#if SB64_MEM_OPT
    thirty_two_width_picture_buffer_desc_init_data.max_width    = sb_size;
    thirty_two_width_picture_buffer_desc_init_data.max_height   = sb_size;
#else
    thirty_two_width_picture_buffer_desc_init_data.max_width    = MAX_SB_SIZE;
    thirty_two_width_picture_buffer_desc_init_data.max_height   = MAX_SB_SIZE;
#endif
    thirty_two_width_picture_buffer_desc_init_data.bit_depth    = EB_32BIT;
    thirty_two_width_picture_buffer_desc_init_data.color_format = EB_YUV420;
    thirty_two_width_picture_buffer_desc_init_data.buffer_enable_mask =
#if MEM_OPT_UV_MODE
        buffer_desc_mask;
#else
        PICTURE_BUFFER_DESC_FULL_MASK;
#endif
    thirty_two_width_picture_buffer_desc_init_data.left_padding  = 0;
    thirty_two_width_picture_buffer_desc_init_data.right_padding = 0;
    thirty_two_width_picture_buffer_desc_init_data.top_padding   = 0;
    thirty_two_width_picture_buffer_desc_init_data.bot_padding   = 0;
    thirty_two_width_picture_buffer_desc_init_data.split_mode    = EB_FALSE;

    // Candidate Ptr
    buffer_ptr->candidate_ptr = (ModeDecisionCandidate *)NULL;

    // Video Buffers
    EB_NEW(buffer_ptr->prediction_ptr,
           eb_picture_buffer_desc_ctor,
           (EbPtr)&picture_buffer_desc_init_data);
#if !CAND_MEM_OPT
    EB_NEW(buffer_ptr->prediction_ptr_temp,
           eb_picture_buffer_desc_ctor,
           (EbPtr)&picture_buffer_desc_init_data);

    EB_NEW(buffer_ptr->cfl_temp_prediction_ptr,
           eb_picture_buffer_desc_ctor,
           (EbPtr)&picture_buffer_desc_init_data);
#endif
#if !MEM_OPT_MD_BUF_DESC
    EB_NEW(buffer_ptr->residual_ptr,
           eb_picture_buffer_desc_ctor,
           (EbPtr)&double_width_picture_buffer_desc_init_data);
#else
    // Reuse the residual_ptr memory in MD context
    buffer_ptr->residual_ptr = temp_residual_ptr;
#endif
#if !CAND_MEM_OPT
    EB_NEW(buffer_ptr->residual_quant_coeff_ptr,
           eb_picture_buffer_desc_ctor,
           (EbPtr)&thirty_two_width_picture_buffer_desc_init_data);
#endif
    EB_NEW(buffer_ptr->recon_coeff_ptr,
           eb_picture_buffer_desc_ctor,
           (EbPtr)&thirty_two_width_picture_buffer_desc_init_data);
#if !MEM_OPT_MD_BUF_DESC
    EB_NEW(
        buffer_ptr->recon_ptr, eb_picture_buffer_desc_ctor, (EbPtr)&picture_buffer_desc_init_data);
#else
    // Reuse the recon_ptr memory in MD context
    buffer_ptr->recon_ptr = temp_recon_ptr;
#endif

    // Costs
    buffer_ptr->fast_cost_ptr       = fast_cost_ptr;
    buffer_ptr->full_cost_ptr       = full_cost_ptr;
    buffer_ptr->full_cost_skip_ptr  = full_cost_skip_ptr;
    buffer_ptr->full_cost_merge_ptr = full_cost_merge_ptr;
    return EB_ErrorNone;
}
EbErrorType mode_decision_scratch_candidate_buffer_ctor(ModeDecisionCandidateBuffer *buffer_ptr,
#if SB64_MEM_OPT
                                                        uint8_t                      sb_size,
#endif
                                                        EbBitDepthEnum               max_bitdepth) {
    EbPictureBufferDescInitData picture_buffer_desc_init_data;
    EbPictureBufferDescInitData double_width_picture_buffer_desc_init_data;
    EbPictureBufferDescInitData thirty_two_width_picture_buffer_desc_init_data;

    buffer_ptr->dctor = mode_decision_scratch_candidate_buffer_dctor;

    // Init Picture Data
#if SB64_MEM_OPT
    picture_buffer_desc_init_data.max_width                       = sb_size;
    picture_buffer_desc_init_data.max_height                      = sb_size;
#else
    picture_buffer_desc_init_data.max_width                       = MAX_SB_SIZE;
    picture_buffer_desc_init_data.max_height                      = MAX_SB_SIZE;
#endif
    picture_buffer_desc_init_data.bit_depth                       = max_bitdepth;
    picture_buffer_desc_init_data.color_format                    = EB_YUV420;
    picture_buffer_desc_init_data.buffer_enable_mask              = PICTURE_BUFFER_DESC_FULL_MASK;
    picture_buffer_desc_init_data.left_padding                    = 0;
    picture_buffer_desc_init_data.right_padding                   = 0;
    picture_buffer_desc_init_data.top_padding                     = 0;
    picture_buffer_desc_init_data.bot_padding                     = 0;
    picture_buffer_desc_init_data.split_mode                      = EB_FALSE;
#if SB64_MEM_OPT
    double_width_picture_buffer_desc_init_data.max_width          = sb_size;
    double_width_picture_buffer_desc_init_data.max_height         = sb_size;
#else
    double_width_picture_buffer_desc_init_data.max_width          = MAX_SB_SIZE;
    double_width_picture_buffer_desc_init_data.max_height         = MAX_SB_SIZE;
#endif
    double_width_picture_buffer_desc_init_data.bit_depth          = EB_16BIT;
    double_width_picture_buffer_desc_init_data.color_format       = EB_YUV420;
    double_width_picture_buffer_desc_init_data.buffer_enable_mask = PICTURE_BUFFER_DESC_FULL_MASK;
    double_width_picture_buffer_desc_init_data.left_padding       = 0;
    double_width_picture_buffer_desc_init_data.right_padding      = 0;
    double_width_picture_buffer_desc_init_data.top_padding        = 0;
    double_width_picture_buffer_desc_init_data.bot_padding        = 0;
    double_width_picture_buffer_desc_init_data.split_mode         = EB_FALSE;
#if SB64_MEM_OPT
    thirty_two_width_picture_buffer_desc_init_data.max_width    = sb_size;
    thirty_two_width_picture_buffer_desc_init_data.max_height   = sb_size;
#else
    thirty_two_width_picture_buffer_desc_init_data.max_width    = MAX_SB_SIZE;
    thirty_two_width_picture_buffer_desc_init_data.max_height   = MAX_SB_SIZE;
#endif
    thirty_two_width_picture_buffer_desc_init_data.bit_depth    = EB_32BIT;
    thirty_two_width_picture_buffer_desc_init_data.color_format = EB_YUV420;
    thirty_two_width_picture_buffer_desc_init_data.buffer_enable_mask =
        PICTURE_BUFFER_DESC_FULL_MASK;
    thirty_two_width_picture_buffer_desc_init_data.left_padding  = 0;
    thirty_two_width_picture_buffer_desc_init_data.right_padding = 0;
    thirty_two_width_picture_buffer_desc_init_data.top_padding   = 0;
    thirty_two_width_picture_buffer_desc_init_data.bot_padding   = 0;
    thirty_two_width_picture_buffer_desc_init_data.split_mode    = EB_FALSE;

    // Candidate Ptr
    buffer_ptr->candidate_ptr = (ModeDecisionCandidate *)NULL;

    // Video Buffers
    EB_NEW(buffer_ptr->prediction_ptr,
           eb_picture_buffer_desc_ctor,
           (EbPtr)&picture_buffer_desc_init_data);
#if ! CAND_MEM_OPT
    EB_NEW(buffer_ptr->prediction_ptr_temp,
           eb_picture_buffer_desc_ctor,
           (EbPtr)&picture_buffer_desc_init_data);

    EB_NEW(buffer_ptr->cfl_temp_prediction_ptr,
           eb_picture_buffer_desc_ctor,
           (EbPtr)&picture_buffer_desc_init_data);
#endif
    EB_NEW(buffer_ptr->residual_ptr,
           eb_picture_buffer_desc_ctor,
           (EbPtr)&double_width_picture_buffer_desc_init_data);
#if ! CAND_MEM_OPT
    EB_NEW(buffer_ptr->residual_quant_coeff_ptr,
           eb_picture_buffer_desc_ctor,
           (EbPtr)&thirty_two_width_picture_buffer_desc_init_data);
#endif
    EB_NEW(buffer_ptr->recon_coeff_ptr,
           eb_picture_buffer_desc_ctor,
           (EbPtr)&thirty_two_width_picture_buffer_desc_init_data);

    EB_NEW(
        buffer_ptr->recon_ptr, eb_picture_buffer_desc_ctor, (EbPtr)&picture_buffer_desc_init_data);
    return EB_ErrorNone;
}
#if !REMOVE_REF_FOR_RECT_PART
uint8_t check_ref_beackout(struct ModeDecisionContext *context_ptr, uint8_t ref_frame_type,
                           Part shape) {
    uint8_t       skip_candidate     = 0;
    const uint8_t allowed_nsq_ref_th = (uint8_t)PRUNE_REC_TH;
    if (context_ptr->prune_ref_frame_for_rec_partitions) {
        if (shape != PART_N) {
            assert(ref_frame_type < 30);
            uint8_t ref_cnt = 0;
            for (uint8_t ref_idx = 0; ref_idx < allowed_nsq_ref_th; ref_idx++)
                if (ref_frame_type == context_ptr->ref_best_ref_sq_table[ref_idx]) {
                    ref_cnt++;
                }
            skip_candidate = !ref_cnt;
        }
    }
    return skip_candidate;
}
#endif
/***************************************
* return true if the MV candidate is already injected
***************************************/
EbBool mrp_is_already_injected_mv_l0(ModeDecisionContext *context_ptr, int16_t mv_x, int16_t mv_y,
                                     uint8_t ref_type) {
    for (int inter_candidate_index = 0; inter_candidate_index < context_ptr->injected_mv_count_l0;
         inter_candidate_index++) {
        if (context_ptr->injected_mv_x_l0_array[inter_candidate_index] == mv_x &&
            context_ptr->injected_mv_y_l0_array[inter_candidate_index] == mv_y &&
            context_ptr->injected_ref_type_l0_array[inter_candidate_index] == ref_type) {
            return (EB_TRUE);
        }
    }

    return (EB_FALSE);
}

EbBool mrp_is_already_injected_mv_l1(ModeDecisionContext *context_ptr, int16_t mv_x, int16_t mv_y,
                                     uint8_t ref_type) {
    for (int inter_candidate_index = 0; inter_candidate_index < context_ptr->injected_mv_count_l1;
         inter_candidate_index++) {
        if (context_ptr->injected_mv_x_l1_array[inter_candidate_index] == mv_x &&
            context_ptr->injected_mv_y_l1_array[inter_candidate_index] == mv_y &&
            context_ptr->injected_ref_type_l1_array[inter_candidate_index] == ref_type) {
            return (EB_TRUE);
        }
    }

    return (EB_FALSE);
}

EbBool mrp_is_already_injected_mv_bipred(ModeDecisionContext *context_ptr, int16_t mv_x_l0,
                                         int16_t mv_y_l0, int16_t mv_x_l1, int16_t mv_y_l1,
                                         uint8_t ref_type) {
    for (int inter_candidate_index = 0;
         inter_candidate_index < context_ptr->injected_mv_count_bipred;
         inter_candidate_index++) {
        if (context_ptr->injected_mv_x_bipred_l0_array[inter_candidate_index] == mv_x_l0 &&
            context_ptr->injected_mv_y_bipred_l0_array[inter_candidate_index] == mv_y_l0 &&
            context_ptr->injected_mv_x_bipred_l1_array[inter_candidate_index] == mv_x_l1 &&
            context_ptr->injected_mv_y_bipred_l1_array[inter_candidate_index] == mv_y_l1 &&
            context_ptr->injected_ref_type_bipred_array[inter_candidate_index] == ref_type) {
            return (EB_TRUE);
        }
    }
    return (EB_FALSE);
}

#if PRUNING_PER_INTER_TYPE
EbBool is_valid_unipred_ref(
    struct ModeDecisionContext *context_ptr,
    uint8_t inter_cand_group,
    uint8_t list_idx, uint8_t ref_idx) {
#if OPT_3
    if (!context_ptr->ref_pruning_ctrls.enabled)
        return EB_TRUE;
#endif
    if (!context_ptr->ref_filtering_res[inter_cand_group][list_idx][ref_idx].do_ref && (ref_idx || !context_ptr->ref_pruning_ctrls.closest_refs[inter_cand_group])) {
        return EB_FALSE;
    }
    else {
        return EB_TRUE;
    }
}

EbBool is_valid_bipred_ref(
    struct ModeDecisionContext *context_ptr,
    uint8_t inter_cand_group,
    uint8_t list_idx_0, uint8_t ref_idx_0,
    uint8_t list_idx_1, uint8_t ref_idx_1) {
#if OPT_3
    if (!context_ptr->ref_pruning_ctrls.enabled)
        return EB_TRUE;
#endif
    // Both ref should be 1 for bipred refs to be valid: if 1 is not best_refs then there is a chance to exit the injection
    if (!context_ptr->ref_filtering_res[inter_cand_group][list_idx_0][ref_idx_0].do_ref ||
        !context_ptr->ref_filtering_res[inter_cand_group][list_idx_1][ref_idx_1].do_ref )
    {
        // Check whether we should check the closest, if no then there no need to move forward and return false
        if (!context_ptr->ref_pruning_ctrls.closest_refs[inter_cand_group])
            return EB_FALSE;

        // Else check if ref are LAST and BWD, if not then return false
        if (ref_idx_0 || ref_idx_1)
            return EB_FALSE;
    }
    return EB_TRUE;
}
#endif
#define BIPRED_3x3_REFINMENT_POSITIONS 8

int8_t allow_refinement_flag[BIPRED_3x3_REFINMENT_POSITIONS] = {1, 0, 1, 0, 1, 0, 1, 0};
int8_t bipred_3x3_x_pos[BIPRED_3x3_REFINMENT_POSITIONS]      = {-1, -1, 0, 1, 1, 1, 0, -1};
int8_t bipred_3x3_y_pos[BIPRED_3x3_REFINMENT_POSITIONS]      = {0, 1, 1, 1, 0, -1, -1, -1};

void unipred_3x3_candidates_injection(const SequenceControlSet *scs_ptr, PictureControlSet *pcs_ptr,
                                      ModeDecisionContext *context_ptr, SuperBlock *sb_ptr,
                                      uint32_t me_sb_addr, uint32_t *candidate_total_cnt) {
    UNUSED(sb_ptr);
    uint32_t           bipred_index;
    uint32_t           cand_total_cnt = (*candidate_total_cnt);
    FrameHeader *      frm_hdr        = &pcs_ptr->parent_pcs_ptr->frm_hdr;
#if DECOUPLE_ME_RES
    MeSbResults *me_results = pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[me_sb_addr];
#else
    const MeSbResults *me_results     = pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr];
#endif
    uint8_t total_me_cnt = me_results->total_me_candidate_index[context_ptr->me_block_offset];
#if ME_MEM_OPT
    const MeCandidate *me_block_results = &me_results->me_candidate_array[context_ptr->me_cand_offset];
#else
    const MeCandidate *me_block_results = me_results->me_candidate[context_ptr->me_block_offset];
#endif
    ModeDecisionCandidate *cand_array   = context_ptr->fast_candidate_array;
    EbBool       is_compound_enabled    = (frm_hdr->reference_mode == SINGLE_REFERENCE) ? 0 : 1;
    IntMv        best_pred_mv[2]        = {{0}, {0}};
    int          inside_tile            = 1;
    MacroBlockD *xd                     = context_ptr->blk_ptr->av1xd;
    int          umv0tile               = (scs_ptr->static_config.unrestricted_motion_vector == 0);
    uint32_t     mi_row                 = context_ptr->blk_origin_y >> MI_SIZE_LOG2;
    uint32_t     mi_col                 = context_ptr->blk_origin_x >> MI_SIZE_LOG2;

    // (8 Best_L0 neighbors)
#if !PRUNING_PER_INTER_TYPE
#if ADD_BEST_CAND_COUNT_SIGNAL
    total_me_cnt = MIN(total_me_cnt, context_ptr->bipred3x3_number_input_mv);
#else
    total_me_cnt = MIN(total_me_cnt, BEST_CANDIDATE_COUNT);
#endif
#endif
    for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; ++me_candidate_index) {
        const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
        const uint8_t      inter_direction      = me_block_results_ptr->direction;
        const uint8_t      list0_ref_index      = me_block_results_ptr->ref_idx_l0;
#if UNIPRED_3x3_REF_MASKING
#if PRUNING_PER_INTER_TYPE
        if (inter_direction == 0) {
        if (!is_valid_unipred_ref(context_ptr, MIN(TOT_INTER_GROUP-1,UNI_3x3_GROUP), REF_LIST_0, list0_ref_index)) continue;
#else
        if (!context_ptr->ref_filtering_res[REF_LIST_0][list0_ref_index].do_ref) continue;
#endif
#endif
#if !PRUNING_PER_INTER_TYPE
        if (inter_direction == 0) {
#endif
            if (list0_ref_index > context_ptr->md_max_ref_count - 1)
                continue;

            for (bipred_index = 0; bipred_index < BIPRED_3x3_REFINMENT_POSITIONS; ++bipred_index) {
                /**************
        NEWMV L0
        ************* */
                if (context_ptr->unipred3x3_injection >= 2) {
                    if (allow_refinement_flag[bipred_index] == 0) continue;
                }
                int16_t to_inject_mv_x;
                int16_t to_inject_mv_y;
                if (pcs_ptr->parent_pcs_ptr->frm_hdr.allow_high_precision_mv) {
                    to_inject_mv_x = context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                        [REF_LIST_0][list0_ref_index][0] +
                        bipred_3x3_x_pos[bipred_index];
                    to_inject_mv_y = context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                        [REF_LIST_0][list0_ref_index][1] +
                        bipred_3x3_y_pos[bipred_index];
                }
                else {
                    to_inject_mv_x = context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                        [REF_LIST_0][list0_ref_index][0] +
                        (bipred_3x3_x_pos[bipred_index] << 1);
                    to_inject_mv_y = context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                        [REF_LIST_0][list0_ref_index][1] +
                        (bipred_3x3_y_pos[bipred_index] << 1);
                }
                uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
#if !REMOVE_REF_FOR_RECT_PART
                uint8_t skip_cand          = check_ref_beackout(
                    context_ptr, to_inject_ref_type, context_ptr->blk_geom->shape);
#endif

                inside_tile = 1;
                if (umv0tile)
                    inside_tile = is_inside_tile_boundary(&(xd->tile),
                                                          to_inject_mv_x,
                                                          to_inject_mv_y,
                                                          mi_col,
                                                          mi_row,
                                                          context_ptr->blk_geom->bsize);
#if REMOVE_REF_FOR_RECT_PART
                uint8_t skip_cand = (!inside_tile);
#else
                skip_cand = skip_cand || (!inside_tile);
#endif

#if INTRA_COMPOUND_OPT
                    MvReferenceFrame rf[2];
                    rf[0] = to_inject_ref_type;
                    rf[1] = -1;
#endif
                if (!skip_cand &&
                    (context_ptr->injected_mv_count_l0 == 0 ||
                     mrp_is_already_injected_mv_l0(
                         context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) ==
                         EB_FALSE)) {
                    uint8_t inter_type;
                    uint8_t is_ii_allowed =
#if INTRA_COMPOUND_OPT
                        svt_is_interintra_allowed(context_ptr->md_inter_intra_level == 1 , context_ptr->blk_geom->bsize, NEWMV, rf);
#else
                        0; //svt_is_interintra_allowed(pcs_ptr->parent_pcs_ptr->enable_inter_intra, bsize, NEWMV, rf);
#endif
#if INTRA_COMPOUND_OPT
                    if (context_ptr->md_inter_intra_level > 2)
#if DECOUPLE_ME_RES
                        if (pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[me_sb_addr]->do_comp[0][list0_ref_index] == 0)
#else
                        if  (pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr]->do_comp[0][list0_ref_index] == 0 )
#endif
                            is_ii_allowed = 0;
#endif
                    uint8_t tot_inter_types = is_ii_allowed ? II_COUNT : 1;
                    //uint8_t is_obmc_allowed =  obmc_motion_mode_allowed(pcs_ptr, context_ptr->blk_ptr, bsize, rf[0], rf[1], NEWMV) == OBMC_CAUSAL;
                    //tot_inter_types = is_obmc_allowed ? tot_inter_types+1 : tot_inter_types;

                    for (inter_type = 0; inter_type < tot_inter_types; inter_type++) {
                        cand_array[cand_total_cnt].type                    = INTER_MODE;
                        cand_array[cand_total_cnt].distortion_ready        = 0;
                        cand_array[cand_total_cnt].use_intrabc             = 0;
                        cand_array[cand_total_cnt].merge_flag              = EB_FALSE;
                        cand_array[cand_total_cnt].prediction_direction[0] = (EbPredDirection)0;
                        cand_array[cand_total_cnt].inter_mode              = NEWMV;
                        cand_array[cand_total_cnt].pred_mode               = NEWMV;
                        cand_array[cand_total_cnt].motion_mode             = SIMPLE_TRANSLATION;

                        cand_array[cand_total_cnt].is_compound = 0;
                        cand_array[cand_total_cnt].is_new_mv   = 1;

                        cand_array[cand_total_cnt].drl_index = 0;

                        // Set the MV to ME result
                        cand_array[cand_total_cnt].motion_vector_xl0 = to_inject_mv_x;
                        cand_array[cand_total_cnt].motion_vector_yl0 = to_inject_mv_y;

                        // will be needed later by the rate estimation
                        cand_array[cand_total_cnt].ref_mv_index   = 0;
                        cand_array[cand_total_cnt].ref_frame_type =
                            svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
                        cand_array[cand_total_cnt].ref_frame_index_l0 = list0_ref_index;
                        cand_array[cand_total_cnt].ref_frame_index_l1 = -1;

                        cand_array[cand_total_cnt].transform_type[0] = DCT_DCT;
                        cand_array[cand_total_cnt].transform_type_uv = DCT_DCT;

                        choose_best_av1_mv_pred(context_ptr,
                                                cand_array[cand_total_cnt].md_rate_estimation_ptr,
                                                context_ptr->blk_ptr,
                                                cand_array[cand_total_cnt].ref_frame_type,
                                                cand_array[cand_total_cnt].is_compound,
                                                cand_array[cand_total_cnt].pred_mode,
                                                cand_array[cand_total_cnt].motion_vector_xl0,
                                                cand_array[cand_total_cnt].motion_vector_yl0,
                                                0,
                                                0,
                                                &cand_array[cand_total_cnt].drl_index,
                                                best_pred_mv);

                        cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_0] =
                            best_pred_mv[0].as_mv.col;
                        cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_0] =
                            best_pred_mv[0].as_mv.row;
                        if (inter_type == 0) {
                            cand_array[cand_total_cnt].is_interintra_used = 0;
                            cand_array[cand_total_cnt].motion_mode        = SIMPLE_TRANSLATION;
                        } else {
                            if (is_ii_allowed) {
                                if (inter_type == 1) {
                                    inter_intra_search(
                                        pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);
                                    cand_array[cand_total_cnt].is_interintra_used   = 1;
                                    cand_array[cand_total_cnt].use_wedge_interintra = 1;
                                } else if (inter_type == 2) {
                                    cand_array[cand_total_cnt].is_interintra_used = 1;
                                    cand_array[cand_total_cnt].interintra_mode =
                                        cand_array[cand_total_cnt - 1].interintra_mode;
                                    cand_array[cand_total_cnt].use_wedge_interintra = 0;
                                }
                            }
                            //if (is_obmc_allowed && inter_type == tot_inter_types - 1) {
                            //    cand_array[cand_total_cnt].is_interintra_used = 0;
                            //    cand_array[cand_total_cnt].motion_mode = OBMC_CAUSAL;
                            //}
                        }

                        INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                    }
                    context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] =
                        to_inject_mv_x;
                    context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] =
                        to_inject_mv_y;
                    context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] =
                        to_inject_ref_type;
                    ++context_ptr->injected_mv_count_l0;
                }
            }
        }
    }

    // (8 Best_L1 neighbors)
#if !PRUNING_PER_INTER_TYPE
#if ADD_BEST_CAND_COUNT_SIGNAL
    total_me_cnt = MIN(total_me_cnt, context_ptr->bipred3x3_number_input_mv);
#else
    total_me_cnt = MIN(total_me_cnt, BEST_CANDIDATE_COUNT);
#endif
#endif
    for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; ++me_candidate_index) {
        const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
        const uint8_t      inter_direction      = me_block_results_ptr->direction;
        const uint8_t      list1_ref_index      = me_block_results_ptr->ref_idx_l1;
#if UNIPRED_3x3_REF_MASKING
#if PRUNING_PER_INTER_TYPE
        if (inter_direction == 1) {
        if (!is_valid_unipred_ref(context_ptr, MIN(TOT_INTER_GROUP-1,UNI_3x3_GROUP), REF_LIST_1, list1_ref_index)) continue;
#else
        if (!context_ptr->ref_filtering_res[REF_LIST_1][list1_ref_index].do_ref) continue;
#endif
#endif
#if !PRUNING_PER_INTER_TYPE
        if (inter_direction == 1) {
#endif
            if (list1_ref_index > context_ptr->md_max_ref_count - 1)
                continue;

            for (bipred_index = 0; bipred_index < BIPRED_3x3_REFINMENT_POSITIONS; ++bipred_index) {
                if (is_compound_enabled) {
                    /**************
            NEWMV L1
            ************* */
                    if (context_ptr->unipred3x3_injection >= 2) {
                        if (allow_refinement_flag[bipred_index] == 0) continue;
                    }
                    int16_t to_inject_mv_x;
                    int16_t to_inject_mv_y;
                    if (pcs_ptr->parent_pcs_ptr->frm_hdr.allow_high_precision_mv) {
                        to_inject_mv_x = context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                            [REF_LIST_1][list1_ref_index][0] +
                            bipred_3x3_x_pos[bipred_index];
                        to_inject_mv_y = context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                            [REF_LIST_1][list1_ref_index][1] +
                            bipred_3x3_y_pos[bipred_index];
                    }
                    else {
                        to_inject_mv_x = context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                            [REF_LIST_1][list1_ref_index][0] +
                            (bipred_3x3_x_pos[bipred_index] << 1);
                        to_inject_mv_y = context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                            [REF_LIST_1][list1_ref_index][1] +
                            (bipred_3x3_y_pos[bipred_index] << 1);
                    }
                    uint8_t to_inject_ref_type =
                        svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
#if !REMOVE_REF_FOR_RECT_PART
                    uint8_t skip_cand = check_ref_beackout(
                        context_ptr, to_inject_ref_type, context_ptr->blk_geom->shape);
#endif

                    inside_tile = 1;
                    if (umv0tile)
                        inside_tile = is_inside_tile_boundary(&(xd->tile),
                                                              to_inject_mv_x,
                                                              to_inject_mv_y,
                                                              mi_col,
                                                              mi_row,
                                                              context_ptr->blk_geom->bsize);
#if REMOVE_REF_FOR_RECT_PART
                    uint8_t skip_cand = (!inside_tile);
#else
                    skip_cand = skip_cand || (!inside_tile);
#endif

#if INTRA_COMPOUND_OPT
                    MvReferenceFrame rf[2];
                    rf[0] = to_inject_ref_type;
                    rf[1] = -1;
#endif
                    if (!skip_cand &&
                        (context_ptr->injected_mv_count_l1 == 0 ||
                         mrp_is_already_injected_mv_l1(
                             context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) ==
                             EB_FALSE)) {
                        uint8_t inter_type;
#if INTRA_COMPOUND_OPT
                        uint8_t is_ii_allowed   = svt_is_interintra_allowed(context_ptr->md_inter_intra_level == 1,
                            context_ptr->blk_geom->bsize, NEWMV, rf);
#else
                        uint8_t is_ii_allowed   = 0;
#endif
#if INTRA_COMPOUND_OPT
                    if (context_ptr->md_inter_intra_level > 2)
#if DECOUPLE_ME_RES
                        if (pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[me_sb_addr]->do_comp[1][list1_ref_index] == 0)
#else
                        if  ( pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr]->do_comp[1][list1_ref_index] == 0)
#endif
                                is_ii_allowed = 0;
#endif
                        uint8_t tot_inter_types = is_ii_allowed ? II_COUNT : 1;
                        for (inter_type = 0; inter_type < tot_inter_types; inter_type++) {
                            cand_array[cand_total_cnt].type                    = INTER_MODE;
                            cand_array[cand_total_cnt].distortion_ready        = 0;
                            cand_array[cand_total_cnt].use_intrabc             = 0;
                            cand_array[cand_total_cnt].merge_flag              = EB_FALSE;
                            cand_array[cand_total_cnt].prediction_direction[0] = (EbPredDirection)1;
                            cand_array[cand_total_cnt].inter_mode              = NEWMV;
                            cand_array[cand_total_cnt].pred_mode               = NEWMV;
                            cand_array[cand_total_cnt].motion_mode             = SIMPLE_TRANSLATION;

                            cand_array[cand_total_cnt].is_compound = 0;
                            cand_array[cand_total_cnt].is_new_mv   = 1;
                            cand_array[cand_total_cnt].drl_index = 0;

                            // Set the MV to ME result
                            cand_array[cand_total_cnt].motion_vector_xl1 = to_inject_mv_x;
                            cand_array[cand_total_cnt].motion_vector_yl1 = to_inject_mv_y;
                            // will be needed later by the rate estimation
                            cand_array[cand_total_cnt].ref_mv_index   = 0;
                            cand_array[cand_total_cnt].ref_frame_type =
                                svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
                            cand_array[cand_total_cnt].ref_frame_index_l0 = -1;
                            cand_array[cand_total_cnt].ref_frame_index_l1 = list1_ref_index;
                            cand_array[cand_total_cnt].transform_type[0]  = DCT_DCT;
                            cand_array[cand_total_cnt].transform_type_uv  = DCT_DCT;
                            choose_best_av1_mv_pred(
                                context_ptr,
                                cand_array[cand_total_cnt].md_rate_estimation_ptr,
                                context_ptr->blk_ptr,
                                cand_array[cand_total_cnt].ref_frame_type,
                                cand_array[cand_total_cnt].is_compound,
                                cand_array[cand_total_cnt].pred_mode,
                                cand_array[cand_total_cnt].motion_vector_xl1,
                                cand_array[cand_total_cnt].motion_vector_yl1,
                                0,
                                0,
                                &cand_array[cand_total_cnt].drl_index,
                                best_pred_mv);

                            cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_1] =
                                best_pred_mv[0].as_mv.col;
                            cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_1] =
                                best_pred_mv[0].as_mv.row;
                            if (inter_type == 0) {
                                cand_array[cand_total_cnt].is_interintra_used = 0;
                                cand_array[cand_total_cnt].motion_mode        = SIMPLE_TRANSLATION;
                            } else {
                                if (is_ii_allowed) {
                                    if (inter_type == 1) {
                                        inter_intra_search(
                                            pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);
                                        cand_array[cand_total_cnt].is_interintra_used   = 1;
                                        cand_array[cand_total_cnt].use_wedge_interintra = 1;
                                    } else if (inter_type == 2) {
                                        cand_array[cand_total_cnt].is_interintra_used = 1;
                                        cand_array[cand_total_cnt].interintra_mode =
                                            cand_array[cand_total_cnt - 1].interintra_mode;
                                        cand_array[cand_total_cnt].use_wedge_interintra = 0;
                                    }
                                }
                                //if (is_obmc_allowed && inter_type == tot_inter_types - 1) {
                                //    cand_array[cand_total_cnt].is_interintra_used = 0;
                                //    cand_array[cand_total_cnt].motion_mode = OBMC_CAUSAL;
                                //}
                            }

                            INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                        }
                        context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] =
                            to_inject_mv_x;
                        context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] =
                            to_inject_mv_y;
                        context_ptr->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] =
                            to_inject_ref_type;
                        ++context_ptr->injected_mv_count_l1;
                    }
                }
            }
        }
    }

    // update the total number of candidates injected
    (*candidate_total_cnt) = cand_total_cnt;

    return;
}

void bipred_3x3_candidates_injection(const SequenceControlSet *scs_ptr, PictureControlSet *pcs_ptr,
                                     ModeDecisionContext *context_ptr, SuperBlock *sb_ptr,
                                     uint32_t me_sb_addr, uint32_t *candidate_total_cnt) {
    UNUSED(sb_ptr);
    uint32_t           cand_total_cnt = (*candidate_total_cnt);
    FrameHeader *      frm_hdr        = &pcs_ptr->parent_pcs_ptr->frm_hdr;
#if DECOUPLE_ME_RES
    const MeSbResults *me_results = pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[me_sb_addr];
#else
    const MeSbResults *me_results     = pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr];
#endif
    uint8_t total_me_cnt = me_results->total_me_candidate_index[context_ptr->me_block_offset];
#if ME_MEM_OPT
    const MeCandidate *me_block_results = &me_results->me_candidate_array[context_ptr->me_cand_offset];
#else
    const MeCandidate *me_block_results = me_results->me_candidate[context_ptr->me_block_offset];
#endif
    ModeDecisionCandidate *cand_array   = context_ptr->fast_candidate_array;
    EbBool       is_compound_enabled    = (frm_hdr->reference_mode == SINGLE_REFERENCE) ? 0 : 1;
    IntMv        best_pred_mv[2]        = {{0}, {0}};
    MacroBlockD *xd                     = context_ptr->blk_ptr->av1xd;
    int          umv0tile               = (scs_ptr->static_config.unrestricted_motion_vector == 0);
    uint32_t     mi_row                 = context_ptr->blk_origin_y >> MI_SIZE_LOG2;
    uint32_t     mi_col                 = context_ptr->blk_origin_x >> MI_SIZE_LOG2;
#if !INTER_COMP_REDESIGN
    BlockSize    bsize          = context_ptr->blk_geom->bsize;
#endif
#if INTER_COMP_REDESIGN
    MD_COMP_TYPE       tot_comp_types = context_ptr->compound_types_to_try;
#else
    MD_COMP_TYPE tot_comp_types = (pcs_ptr->parent_pcs_ptr->compound_mode == 1 ||
                                   context_ptr->compound_types_to_try == MD_COMP_AVG)
                                      ? MD_COMP_AVG
                                      : (bsize >= BLOCK_8X8 && bsize <= BLOCK_32X32)
                                            ? context_ptr->compound_types_to_try
                                            : context_ptr->compound_types_to_try == MD_COMP_WEDGE
                                                  ? MD_COMP_DIFF0
                                                  : context_ptr->compound_types_to_try;
#endif
#if !INTER_COMP_REDESIGN

    if (context_ptr->source_variance < context_ptr->inter_inter_wedge_variance_th)
        tot_comp_types = MIN(tot_comp_types, MD_COMP_DIFF0);

#else
#if !OPT_4
    if (context_ptr->source_variance < context_ptr->inter_comp_ctrls.wedge_variance_th)
        tot_comp_types = MIN(tot_comp_types, MD_COMP_DIFF0);
#endif
#endif

    if (is_compound_enabled) {
        /**************
       NEW_NEWMV
       ************* */
#if !PRUNING_PER_INTER_TYPE
#if ADD_BEST_CAND_COUNT_SIGNAL
        total_me_cnt = MIN(total_me_cnt, context_ptr->bipred3x3_number_input_mv);
#else
        total_me_cnt = MIN(total_me_cnt, BEST_CANDIDATE_COUNT);
#endif
#endif
        for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt;
             ++me_candidate_index) {
            const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
            const uint8_t      inter_direction      = me_block_results_ptr->direction;
            const uint8_t      list0_ref_index      = me_block_results_ptr->ref_idx_l0;
            const uint8_t      list1_ref_index      = me_block_results_ptr->ref_idx_l1;
#if BIPRED_3x3_REF_MASKING
#if PRUNING_PER_INTER_TYPE
           if (inter_direction == 2) {
           if (!is_valid_bipred_ref(
                context_ptr, BI_3x3_GROUP, me_block_results_ptr->ref0_list, list0_ref_index, me_block_results_ptr->ref1_list, list1_ref_index)) continue;
#else
            if (!context_ptr->ref_filtering_res[me_block_results_ptr->ref0_list][list0_ref_index].do_ref || !context_ptr->ref_filtering_res[me_block_results_ptr->ref1_list][list1_ref_index].do_ref) continue;
#endif
#endif
#if !PRUNING_PER_INTER_TYPE
            if (inter_direction == 2) {
#endif
                if (list0_ref_index > context_ptr->md_max_ref_count - 1 ||
                    list1_ref_index > context_ptr->md_max_ref_count - 1)
                    continue;
                // (Best_L0, 8 Best_L1 neighbors)
                for (uint32_t bipred_index = 0; bipred_index < BIPRED_3x3_REFINMENT_POSITIONS;
                     ++bipred_index) {
                    if (context_ptr->bipred3x3_injection >= 2) {
                        if (allow_refinement_flag[bipred_index] == 0) continue;
                    }
                    int16_t to_inject_mv_x_l0 =
                        context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                        [me_block_results_ptr->ref0_list][list0_ref_index][0];
                    int16_t to_inject_mv_y_l0 =
                        context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                        [me_block_results_ptr->ref0_list][list0_ref_index][1];

                    int16_t to_inject_mv_x_l1;
                    int16_t to_inject_mv_y_l1;
                    if (pcs_ptr->parent_pcs_ptr->frm_hdr.allow_high_precision_mv) {
                        to_inject_mv_x_l1 =
                            context_ptr
                            ->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                            [me_block_results_ptr->ref1_list][list1_ref_index][0] +
                            bipred_3x3_x_pos[bipred_index];
                        to_inject_mv_y_l1 =
                            context_ptr
                            ->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                            [me_block_results_ptr->ref1_list][list1_ref_index][1] +
                            bipred_3x3_y_pos[bipred_index];
                    }
                    else {
                        to_inject_mv_x_l1 =
                            context_ptr
                            ->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                            [me_block_results_ptr->ref1_list][list1_ref_index][0] +
                            (bipred_3x3_x_pos[bipred_index] << 1);
                        to_inject_mv_y_l1 =
                            context_ptr
                            ->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                            [me_block_results_ptr->ref1_list][list1_ref_index][1] +
                            (bipred_3x3_y_pos[bipred_index] << 1);
                    }

                    uint8_t to_inject_ref_type = av1_ref_frame_type((const MvReferenceFrame[]){
                        svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index),
                        svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index)});
#if !REMOVE_REF_FOR_RECT_PART
                    uint8_t skip_cand          = check_ref_beackout(
                        context_ptr, to_inject_ref_type, context_ptr->blk_geom->shape);
#endif

                    int inside_tile = umv0tile ? is_inside_tile_boundary(&(xd->tile),
                                                              to_inject_mv_x_l0,
                                                              to_inject_mv_y_l0,
                                                              mi_col,
                                                              mi_row,
                                                              context_ptr->blk_geom->bsize)
                                               : 1;
#if REMOVE_REF_FOR_RECT_PART
                    uint8_t skip_cand = (!inside_tile);
#else
                    skip_cand = skip_cand || (!inside_tile);
#endif
                    if (!skip_cand &&
                        (context_ptr->injected_mv_count_bipred == 0 ||
                         mrp_is_already_injected_mv_bipred(context_ptr,
                                                           to_inject_mv_x_l0,
                                                           to_inject_mv_y_l0,
                                                           to_inject_mv_x_l1,
                                                           to_inject_mv_y_l1,
                                                           to_inject_ref_type) == EB_FALSE)) {
#if !INTER_COMP_REDESIGN
                        context_ptr->variance_ready = 0;
#endif
#if INTER_COMP_REDESIGN
                        if (context_ptr->inter_comp_ctrls.mrp_pruning_w_distortion)
#if DECOUPLE_ME_RES
                            if (pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[me_sb_addr]->do_comp[0][list0_ref_index] == 0 ||
                                pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[me_sb_addr]->do_comp[1][list1_ref_index] == 0)
                                tot_comp_types = MD_COMP_AVG;
#else
                            if  (pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr]->do_comp[0][list0_ref_index] == 0 ||
                                 pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr]->do_comp[1][list1_ref_index] == 0)
                                    tot_comp_types = MD_COMP_AVG;
#endif
#endif
                        for (MD_COMP_TYPE cur_type = MD_COMP_AVG; cur_type <= tot_comp_types; cur_type++) {
#if !INTER_COMP_REDESIGN
                            if (cur_type == MD_COMP_WEDGE &&
                                    get_wedge_params_bits(context_ptr->blk_geom->bsize) == 0)
                                continue;
#endif
#if !REMOVE_SIMILARITY_FEATS

                            // If two predictors are very similar, skip wedge compound mode search
#if INTER_COMP_REDESIGN
                            if (cur_type == MD_COMP_WEDGE && context_ptr->inter_comp_ctrls.similar_predictions)
#else
                            if (context_ptr->variance_ready)
#endif
                                if (context_ptr->prediction_mse < 8)
                                    continue;
#endif
                            cand_array[cand_total_cnt].type             = INTER_MODE;
                            cand_array[cand_total_cnt].distortion_ready = 0;
                            cand_array[cand_total_cnt].use_intrabc      = 0;
                            cand_array[cand_total_cnt].merge_flag       = EB_FALSE;
                            cand_array[cand_total_cnt].is_new_mv        = 1;
                            cand_array[cand_total_cnt].drl_index = 0;

                            // Set the MV to ME result
                            cand_array[cand_total_cnt].motion_vector_xl0 = to_inject_mv_x_l0;
                            cand_array[cand_total_cnt].motion_vector_yl0 = to_inject_mv_y_l0;
                            cand_array[cand_total_cnt].motion_vector_xl1 = to_inject_mv_x_l1;
                            cand_array[cand_total_cnt].motion_vector_yl1 = to_inject_mv_y_l1;
                            // will be needed later by the rate estimation
                            cand_array[cand_total_cnt].ref_mv_index            = 0;
                            cand_array[cand_total_cnt].inter_mode              = NEW_NEWMV;
                            cand_array[cand_total_cnt].pred_mode               = NEW_NEWMV;
                            cand_array[cand_total_cnt].motion_mode             = SIMPLE_TRANSLATION;
                            cand_array[cand_total_cnt].is_compound             = 1;
                            cand_array[cand_total_cnt].is_interintra_used      = 0;
                            cand_array[cand_total_cnt].prediction_direction[0] = (EbPredDirection)2;
                            cand_array[cand_total_cnt].ref_frame_type = av1_ref_frame_type(
                                (const MvReferenceFrame[]){
                                    svt_get_ref_frame_type(me_block_results_ptr->ref0_list,
                                                           list0_ref_index),
                                    svt_get_ref_frame_type(me_block_results_ptr->ref1_list,
                                                           list1_ref_index)});
                            cand_array[cand_total_cnt].ref_frame_index_l0 = list0_ref_index;
                            cand_array[cand_total_cnt].ref_frame_index_l1 = list1_ref_index;
                            cand_array[cand_total_cnt].transform_type[0]  = DCT_DCT;
                            cand_array[cand_total_cnt].transform_type_uv  = DCT_DCT;
                            choose_best_av1_mv_pred(
                                context_ptr,
                                cand_array[cand_total_cnt].md_rate_estimation_ptr,
                                context_ptr->blk_ptr,
                                cand_array[cand_total_cnt].ref_frame_type,
                                cand_array[cand_total_cnt].is_compound,
                                cand_array[cand_total_cnt].pred_mode,
                                cand_array[cand_total_cnt].motion_vector_xl0,
                                cand_array[cand_total_cnt].motion_vector_yl0,
                                cand_array[cand_total_cnt].motion_vector_xl1,
                                cand_array[cand_total_cnt].motion_vector_yl1,
                                &cand_array[cand_total_cnt].drl_index,
                                best_pred_mv);

                            cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_0] =
                                best_pred_mv[0].as_mv.col;
                            cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_0] =
                                best_pred_mv[0].as_mv.row;
                            cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_1] =
                                best_pred_mv[1].as_mv.col;
                            cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_1] =
                                best_pred_mv[1].as_mv.row;
#if INTER_COMP_REDESIGN
                            if (cur_type == MD_COMP_AVG && tot_comp_types > MD_COMP_AVG)
                                calc_pred_masked_compound(
                                    pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);
#if !REMOVE_SIMILARITY_FEATS
                            //BIP 3x3
                            if (context_ptr->inter_comp_ctrls.similar_predictions)
                                if (cur_type > MD_COMP_AVG &&
                                    context_ptr->prediction_mse <=
                                    context_ptr->inter_comp_ctrls.similar_predictions_th )
                                    continue;
#endif
#endif
                            //BIP 3x3
                            determine_compound_mode(
                                pcs_ptr, context_ptr, &cand_array[cand_total_cnt], cur_type);
                            INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                            context_ptr->injected_mv_x_bipred_l0_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                            context_ptr->injected_mv_y_bipred_l0_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                            context_ptr->injected_mv_x_bipred_l1_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                            context_ptr->injected_mv_y_bipred_l1_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                            context_ptr->injected_ref_type_bipred_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_ref_type;
                            ++context_ptr->injected_mv_count_bipred;
                        }
                    }
                }

                // (8 Best_L0 neighbors, Best_L1) :
                for (uint32_t bipred_index = 0; bipred_index < BIPRED_3x3_REFINMENT_POSITIONS;
                     ++bipred_index) {
                    if (context_ptr->bipred3x3_injection >= 2) {
                        if (allow_refinement_flag[bipred_index] == 0) continue;
                    }

                    int16_t to_inject_mv_x_l0;
                    int16_t to_inject_mv_y_l0;
                    if (pcs_ptr->parent_pcs_ptr->frm_hdr.allow_high_precision_mv) {
                        to_inject_mv_x_l0 =
                            context_ptr
                            ->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                            [me_block_results_ptr->ref0_list][list0_ref_index][0] +
                            bipred_3x3_x_pos[bipred_index];
                        to_inject_mv_y_l0 =
                            context_ptr
                            ->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                            [me_block_results_ptr->ref0_list][list0_ref_index][1] +
                            bipred_3x3_y_pos[bipred_index];
                    }
                    else {
                        to_inject_mv_x_l0 =
                            context_ptr
                            ->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                            [me_block_results_ptr->ref0_list][list0_ref_index][0] +
                            (bipred_3x3_x_pos[bipred_index] << 1);
                        to_inject_mv_y_l0 =
                            context_ptr
                            ->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                            [me_block_results_ptr->ref0_list][list0_ref_index][1] +
                            (bipred_3x3_y_pos[bipred_index] << 1);
                    }
                    int16_t to_inject_mv_x_l1 =
                        context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                        [me_block_results_ptr->ref1_list][list1_ref_index][0];
                    int16_t to_inject_mv_y_l1 =
                        context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                        [me_block_results_ptr->ref1_list][list1_ref_index][1];
                    uint8_t to_inject_ref_type = av1_ref_frame_type((const MvReferenceFrame[]){
                        svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index),
                        svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index)});
#if !REMOVE_REF_FOR_RECT_PART
                    uint8_t skip_cand          = check_ref_beackout(
                        context_ptr, to_inject_ref_type, context_ptr->blk_geom->shape);
#endif

                    int inside_tile =  umv0tile ? is_inside_tile_boundary(&(xd->tile),
                                                              to_inject_mv_x_l0,
                                                              to_inject_mv_y_l0,
                                                              mi_col,
                                                              mi_row,
                                                              context_ptr->blk_geom->bsize) &&
                                      is_inside_tile_boundary(&(xd->tile),
                                                              to_inject_mv_x_l1,
                                                              to_inject_mv_y_l1,
                                                              mi_col,
                                                              mi_row,
                                                              context_ptr->blk_geom->bsize)
                                                : 1;
#if REMOVE_REF_FOR_RECT_PART
                    uint8_t skip_cand = (!inside_tile);
#else
                    skip_cand = skip_cand || (!inside_tile);
#endif
                    if (!skip_cand &&
                        (context_ptr->injected_mv_count_bipred == 0 ||
                         mrp_is_already_injected_mv_bipred(context_ptr,
                                                           to_inject_mv_x_l0,
                                                           to_inject_mv_y_l0,
                                                           to_inject_mv_x_l1,
                                                           to_inject_mv_y_l1,
                                                           to_inject_ref_type) == EB_FALSE)) {
#if !INTER_COMP_REDESIGN
                        context_ptr->variance_ready = 0;
#endif
#if INTER_COMP_REDESIGN
#if DECOUPLE_ME_RES
                        if (context_ptr->inter_comp_ctrls.mrp_pruning_w_distortion)
                            if (pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[me_sb_addr]->do_comp[0][list0_ref_index] == 0 ||
                                pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[me_sb_addr]->do_comp[1][list1_ref_index] == 0)
                                tot_comp_types = MD_COMP_AVG;
#else
                        if (context_ptr->inter_comp_ctrls.mrp_pruning_w_distortion)
                            if  (pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr]->do_comp[0][list0_ref_index] == 0 ||
                                 pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr]->do_comp[1][list1_ref_index] == 0)
                                    tot_comp_types = MD_COMP_AVG;
#endif
#endif
                        for (MD_COMP_TYPE cur_type = MD_COMP_AVG; cur_type <= tot_comp_types; cur_type++) {
#if !INTER_COMP_REDESIGN
                            if (cur_type == MD_COMP_WEDGE &&
                                    get_wedge_params_bits(context_ptr->blk_geom->bsize) == 0)
                                continue;
#endif
#if !REMOVE_SIMILARITY_FEATS
                            // If two predictors are very similar, skip wedge compound mode search
#if INTER_COMP_REDESIGN
                            if (cur_type == MD_COMP_WEDGE && context_ptr->inter_comp_ctrls.similar_predictions)
#else
                            if (context_ptr->variance_ready)
#endif
                                if (context_ptr->prediction_mse < 8)
                                    continue;
#endif
                            cand_array[cand_total_cnt].type             = INTER_MODE;
                            cand_array[cand_total_cnt].distortion_ready = 0;
                            cand_array[cand_total_cnt].use_intrabc      = 0;
                            cand_array[cand_total_cnt].merge_flag       = EB_FALSE;

                            cand_array[cand_total_cnt].is_new_mv  = 1;
                            cand_array[cand_total_cnt].drl_index = 0;

                            // Set the MV to ME result
                            cand_array[cand_total_cnt].motion_vector_xl0 = to_inject_mv_x_l0;
                            cand_array[cand_total_cnt].motion_vector_yl0 = to_inject_mv_y_l0;
                            cand_array[cand_total_cnt].motion_vector_xl1 = to_inject_mv_x_l1;
                            cand_array[cand_total_cnt].motion_vector_yl1 = to_inject_mv_y_l1;
                            // will be needed later by the rate estimation
                            cand_array[cand_total_cnt].ref_mv_index            = 0;
                            cand_array[cand_total_cnt].inter_mode              = NEW_NEWMV;
                            cand_array[cand_total_cnt].pred_mode               = NEW_NEWMV;
                            cand_array[cand_total_cnt].motion_mode             = SIMPLE_TRANSLATION;
                            cand_array[cand_total_cnt].is_compound             = 1;
                            cand_array[cand_total_cnt].is_interintra_used      = 0;
                            cand_array[cand_total_cnt].prediction_direction[0] = (EbPredDirection)2;
                            cand_array[cand_total_cnt].ref_frame_type = av1_ref_frame_type(
                                (const MvReferenceFrame[]){
                                    svt_get_ref_frame_type(me_block_results_ptr->ref0_list,
                                                           list0_ref_index),
                                    svt_get_ref_frame_type(me_block_results_ptr->ref1_list,
                                                           list1_ref_index)});
                            cand_array[cand_total_cnt].ref_frame_index_l0 = list0_ref_index;
                            cand_array[cand_total_cnt].ref_frame_index_l1 = list1_ref_index;
                            cand_array[cand_total_cnt].transform_type[0]  = DCT_DCT;
                            cand_array[cand_total_cnt].transform_type_uv  = DCT_DCT;
                            choose_best_av1_mv_pred(
                                context_ptr,
                                cand_array[cand_total_cnt].md_rate_estimation_ptr,
                                context_ptr->blk_ptr,
                                cand_array[cand_total_cnt].ref_frame_type,
                                cand_array[cand_total_cnt].is_compound,
                                cand_array[cand_total_cnt].pred_mode,
                                cand_array[cand_total_cnt].motion_vector_xl0,
                                cand_array[cand_total_cnt].motion_vector_yl0,
                                cand_array[cand_total_cnt].motion_vector_xl1,
                                cand_array[cand_total_cnt].motion_vector_yl1,
                                &cand_array[cand_total_cnt].drl_index,
                                best_pred_mv);

                            cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_0] =
                                best_pred_mv[0].as_mv.col;
                            cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_0] =
                                best_pred_mv[0].as_mv.row;
                            cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_1] =
                                best_pred_mv[1].as_mv.col;
                            cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_1] =
                                best_pred_mv[1].as_mv.row;
#if INTER_COMP_REDESIGN
                            if (cur_type == MD_COMP_AVG && tot_comp_types > MD_COMP_AVG)
                                calc_pred_masked_compound(
                                    pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);
#if !REMOVE_SIMILARITY_FEATS
                            //BIP 3x3
                            if (context_ptr->inter_comp_ctrls.similar_predictions)
                                if (cur_type > MD_COMP_AVG &&
                                    context_ptr->prediction_mse <=
                                    context_ptr->inter_comp_ctrls.similar_predictions_th )
                                    continue;
#endif
#endif
                            //BIP 3x3
                            determine_compound_mode(
                                pcs_ptr, context_ptr, &cand_array[cand_total_cnt], cur_type);
                            INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                            context_ptr->injected_mv_x_bipred_l0_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                            context_ptr->injected_mv_y_bipred_l0_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                            context_ptr->injected_mv_x_bipred_l1_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                            context_ptr->injected_mv_y_bipred_l1_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                            context_ptr->injected_ref_type_bipred_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_ref_type;
                            ++context_ptr->injected_mv_count_bipred;
                        }
                    }
                }
            }
        }
    }

    // update the total number of candidates injected
    (*candidate_total_cnt) = cand_total_cnt;

    return;
}

uint8_t get_max_drl_index(uint8_t refmvCnt, PredictionMode mode) {
    uint8_t max_drl = 0;

    if (mode == NEWMV || mode == NEW_NEWMV) {
        if (refmvCnt < 2)
            max_drl = 1;
        else if (refmvCnt == 2)
            max_drl = 2;
        else
            max_drl = 3;
    }

    if (mode == NEARMV || mode == NEAR_NEARMV || mode == NEAR_NEWMV || mode == NEW_NEARMV) {
        if (refmvCnt < 3)
            max_drl = 1;
        else if (refmvCnt == 3)
            max_drl = 2;
        else
            max_drl = 3;
    }

    return max_drl;
}
/*********************************************************************
**********************************************************************
        Upto 12 inter Candidated injected
        Min 6 inter Candidated injected
UniPred L0 : NEARST         + upto 3x NEAR
UniPred L1 : NEARST         + upto 3x NEAR
BIPred     : NEARST_NEARST  + upto 3x NEAR_NEAR
**********************************************************************
**********************************************************************/
void inject_mvp_candidates_ii(struct ModeDecisionContext *context_ptr, PictureControlSet *pcs_ptr,
                              BlkStruct *blk_ptr, MvReferenceFrame ref_pair,
                              uint32_t *candTotCnt) {
    FrameHeader *frm_hdr = &pcs_ptr->parent_pcs_ptr->frm_hdr;
    EbBool       allow_compound =
        (frm_hdr->reference_mode == SINGLE_REFERENCE || context_ptr->blk_geom->bwidth == 4 ||
         context_ptr->blk_geom->bheight == 4)
            ? EB_FALSE
            : EB_TRUE;
    uint8_t                inj_mv;
    uint32_t               cand_idx   = *candTotCnt;
    ModeDecisionCandidate *cand_array = context_ptr->fast_candidate_array;
    MacroBlockD *          xd         = blk_ptr->av1xd;
    uint8_t                drli, max_drl_index;
    IntMv                  nearestmv[2], nearmv[2], ref_mv[2];

    MvReferenceFrame    rf[2];
    int                 inside_tile = 1;
    SequenceControlSet *scs_ptr =
        (SequenceControlSet *)pcs_ptr->parent_pcs_ptr->scs_wrapper_ptr->object_ptr;
    int      umv0tile = (scs_ptr->static_config.unrestricted_motion_vector == 0);
    uint32_t mi_row   = context_ptr->blk_origin_y >> MI_SIZE_LOG2;
    uint32_t mi_col   = context_ptr->blk_origin_x >> MI_SIZE_LOG2;
    av1_set_ref_frame(rf, ref_pair);
    MD_COMP_TYPE cur_type; //MVP
    BlockSize    bsize          = context_ptr->blk_geom->bsize; // bloc size
#if INTER_COMP_REDESIGN
    MD_COMP_TYPE tot_comp_types =  context_ptr->compound_types_to_try;
#else
    MD_COMP_TYPE tot_comp_types = (bsize >= BLOCK_8X8 && bsize <= BLOCK_32X32)
                                      ? context_ptr->compound_types_to_try
                                      : context_ptr->compound_types_to_try == MD_COMP_WEDGE
                                            ? MD_COMP_DIFF0
                                            : context_ptr->compound_types_to_try;
#endif
#if !INTER_COMP_REDESIGN
    if (context_ptr->source_variance < context_ptr->inter_inter_wedge_variance_th)
        tot_comp_types = MIN(tot_comp_types, MD_COMP_DIFF0);

#else
#if !OPT_4
    if (context_ptr->source_variance < context_ptr->inter_comp_ctrls.wedge_variance_th)
        tot_comp_types = MIN(tot_comp_types, MD_COMP_DIFF0);
#endif
#endif
    //single ref/list
    if (rf[1] == NONE_FRAME) {
        MvReferenceFrame frame_type = rf[0];
        uint8_t          list_idx   = get_list_idx(rf[0]);
        uint8_t          ref_idx    = get_ref_frame_idx(rf[0]);
#if NEAREST_NEAR_REF_MASKING
        // Always consider the 2 closet ref frames (i.e. ref_idx=0) @ MVP cand generation
#if PRUNING_PER_INTER_TYPE
        if (!is_valid_unipred_ref(context_ptr, MIN(TOT_INTER_GROUP-1,NRST_NEAR_GROUP), list_idx, ref_idx)) return;
#else
        if (!context_ptr->ref_filtering_res[list_idx][ref_idx].do_ref && ref_idx) return;
#endif
#endif
        if (ref_idx > context_ptr->md_max_ref_count - 1) return;
        //NEAREST
        int16_t to_inject_mv_x = context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                                     .ref_mvs[frame_type][0]
                                     .as_mv.col;
        int16_t to_inject_mv_y = context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                                     .ref_mvs[frame_type][0]
                                     .as_mv.row;

        inj_mv = list_idx == 0
                     ? context_ptr->injected_mv_count_l0 == 0 ||
                           mrp_is_already_injected_mv_l0(
                               context_ptr, to_inject_mv_x, to_inject_mv_y, frame_type) == EB_FALSE
                     : context_ptr->injected_mv_count_l1 == 0 ||
                           mrp_is_already_injected_mv_l1(
                               context_ptr, to_inject_mv_x, to_inject_mv_y, frame_type) == EB_FALSE;

        if (umv0tile)
            inside_tile = is_inside_tile_boundary(&(xd->tile),
                                                  to_inject_mv_x,
                                                  to_inject_mv_y,
                                                  mi_col,
                                                  mi_row,
                                                  context_ptr->blk_geom->bsize);
        inj_mv = inj_mv && inside_tile;
        if (inj_mv) {
            uint8_t inter_type;
            uint8_t is_ii_allowed =
                svt_is_interintra_allowed(context_ptr->md_inter_intra_level, bsize, NEARESTMV, rf);
#if INTRA_COMPOUND_OPT
            uint8_t ref_idx_0 = get_ref_frame_idx(rf[0]);

            if (context_ptr->md_inter_intra_level > 2)
                if (ref_idx_0 > context_ptr->inter_comp_ctrls.mrp_pruning_w_distance - 1)
                    is_ii_allowed = 0;
#endif
            uint8_t tot_inter_types = is_ii_allowed ? II_COUNT : 1;
            uint8_t is_obmc_allowed =
                obmc_motion_mode_allowed(pcs_ptr, context_ptr, bsize, rf[0], rf[1], NEARESTMV) ==
                OBMC_CAUSAL;
#if OBMC_FAST
#if ON_OFF_FEATURE_MRP
            is_obmc_allowed = context_ptr->obmc_ctrls.enabled == 0 ? 0 :
#else
            is_obmc_allowed = context_ptr->obmc_ctrls.mvp_ref_count == 0 ? 0 :
#endif
                ref_idx > context_ptr->obmc_ctrls.mvp_ref_count - 1 ? 0 : is_obmc_allowed;
#endif
            tot_inter_types = is_obmc_allowed ? tot_inter_types + 1 : tot_inter_types;
            for (inter_type = 0; inter_type < tot_inter_types; inter_type++) {
                cand_array[cand_idx].type                    = INTER_MODE;
                cand_array[cand_idx].inter_mode              = NEARESTMV;
                cand_array[cand_idx].pred_mode               = NEARESTMV;
                cand_array[cand_idx].motion_mode             = SIMPLE_TRANSLATION;
                cand_array[cand_idx].is_compound             = 0;
                cand_array[cand_idx].distortion_ready        = 0;
                cand_array[cand_idx].use_intrabc             = 0;
                cand_array[cand_idx].merge_flag              = EB_FALSE;
                cand_array[cand_idx].prediction_direction[0] = list_idx;
                cand_array[cand_idx].is_new_mv               = 0;
                cand_array[cand_idx].drl_index      = 0;
                cand_array[cand_idx].ref_mv_index   = 0;
                cand_array[cand_idx].ref_frame_type = frame_type;

                cand_array[cand_idx].ref_frame_index_l0 = (list_idx == 0) ? ref_idx : -1;
                cand_array[cand_idx].ref_frame_index_l1 = (list_idx == 1) ? ref_idx : -1;
                cand_array[cand_idx].transform_type[0]  = DCT_DCT;
                cand_array[cand_idx].transform_type_uv  = DCT_DCT;
                if (list_idx == 0) {
                    cand_array[cand_idx].motion_vector_xl0 = to_inject_mv_x;
                    cand_array[cand_idx].motion_vector_yl0 = to_inject_mv_y;
                    context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] =
                        to_inject_mv_x;
                    context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] =
                        to_inject_mv_y;
                    context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] =
                        frame_type;
                    ++context_ptr->injected_mv_count_l0;
                } else {
                    cand_array[cand_idx].motion_vector_xl1 = to_inject_mv_x;
                    cand_array[cand_idx].motion_vector_yl1 = to_inject_mv_y;
                    context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] =
                        to_inject_mv_x;
                    context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] =
                        to_inject_mv_y;
                    context_ptr->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] =
                        frame_type;
                    ++context_ptr->injected_mv_count_l1;
                }
                if (inter_type == 0) {
                    cand_array[cand_idx].is_interintra_used = 0;
                    cand_array[cand_idx].motion_mode        = SIMPLE_TRANSLATION;
                } else {
                    if (is_ii_allowed) {
                        if (inter_type == 1) {
                            inter_intra_search(pcs_ptr, context_ptr, &cand_array[cand_idx]);
                            cand_array[cand_idx].is_interintra_used   = 1;
                            cand_array[cand_idx].use_wedge_interintra = 1;
                        } else if (inter_type == 2) {
                            cand_array[cand_idx].is_interintra_used = 1;
                            cand_array[cand_idx].interintra_mode =
                                cand_array[cand_idx - 1].interintra_mode;
                            cand_array[cand_idx].use_wedge_interintra = 0;
                        }
                    }
                    if (is_obmc_allowed && inter_type == tot_inter_types - 1) {
                        cand_array[cand_idx].is_interintra_used = 0;
                        cand_array[cand_idx].motion_mode        = OBMC_CAUSAL;
                    }
                }

                INCRMENT_CAND_TOTAL_COUNT(cand_idx);
            }
        }

        //NEAR
        max_drl_index = get_max_drl_index(xd->ref_mv_count[frame_type], NEARMV);

        for (drli = 0; drli < max_drl_index; drli++) {
            get_av1_mv_pred_drl(
                context_ptr, blk_ptr, frame_type, 0, NEARMV, drli, nearestmv, nearmv, ref_mv);

            to_inject_mv_x = nearmv[0].as_mv.col;
            to_inject_mv_y = nearmv[0].as_mv.row;

            inj_mv =
                list_idx == 0
                    ? context_ptr->injected_mv_count_l0 == 0 ||
                          mrp_is_already_injected_mv_l0(
                              context_ptr, to_inject_mv_x, to_inject_mv_y, frame_type) == EB_FALSE
                    : context_ptr->injected_mv_count_l1 == 0 ||
                          mrp_is_already_injected_mv_l1(
                              context_ptr, to_inject_mv_x, to_inject_mv_y, frame_type) == EB_FALSE;

            if (umv0tile)
                inside_tile = is_inside_tile_boundary(&(xd->tile),
                                                      to_inject_mv_x,
                                                      to_inject_mv_y,
                                                      mi_col,
                                                      mi_row,
                                                      context_ptr->blk_geom->bsize);
            inj_mv = inj_mv && inside_tile;
            if (inj_mv) {
                uint8_t inter_type;
                uint8_t is_ii_allowed = svt_is_interintra_allowed(
                    context_ptr->md_inter_intra_level, bsize, NEARMV, rf);
#if INTRA_COMPOUND_OPT
            uint8_t ref_idx_0 = get_ref_frame_idx(rf[0]);

            if (context_ptr->md_inter_intra_level > 2)
                if (ref_idx_0 > context_ptr->inter_comp_ctrls.mrp_pruning_w_distance - 1)
                    is_ii_allowed = 0;
#endif
                uint8_t tot_inter_types = is_ii_allowed ? II_COUNT : 1;
                uint8_t is_obmc_allowed =
                    obmc_motion_mode_allowed(pcs_ptr, context_ptr, bsize, rf[0], rf[1], NEARMV) ==
                    OBMC_CAUSAL;
#if OBMC_FAST
#if ON_OFF_FEATURE_MRP
                is_obmc_allowed = context_ptr->obmc_ctrls.enabled == 0 ? 0 :
#else
                is_obmc_allowed = context_ptr->obmc_ctrls.mvp_ref_count == 0 ? 0 :
#endif
                    ref_idx > context_ptr->obmc_ctrls.mvp_ref_count - 1 ? 0 : is_obmc_allowed;
                is_obmc_allowed = context_ptr->obmc_ctrls.near_count == 0 ? 0 :
                    drli > context_ptr->obmc_ctrls.near_count - 1 ? 0 : is_obmc_allowed;
#endif
                tot_inter_types = is_obmc_allowed ? tot_inter_types + 1 : tot_inter_types;
                for (inter_type = 0; inter_type < tot_inter_types; inter_type++) {
                    cand_array[cand_idx].type                    = INTER_MODE;
                    cand_array[cand_idx].inter_mode              = NEARMV;
                    cand_array[cand_idx].pred_mode               = NEARMV;
                    cand_array[cand_idx].motion_mode             = SIMPLE_TRANSLATION;
                    cand_array[cand_idx].is_compound             = 0;
                    cand_array[cand_idx].distortion_ready        = 0;
                    cand_array[cand_idx].use_intrabc             = 0;
                    cand_array[cand_idx].merge_flag              = EB_FALSE;
                    cand_array[cand_idx].prediction_direction[0] = list_idx;
                    cand_array[cand_idx].is_new_mv               = 0;
                    cand_array[cand_idx].drl_index               = drli;
                    cand_array[cand_idx].ref_mv_index            = 0;
                    cand_array[cand_idx].ref_frame_type          = frame_type;

                    cand_array[cand_idx].ref_frame_index_l0 = (list_idx == 0) ? ref_idx : -1;
                    cand_array[cand_idx].ref_frame_index_l1 = (list_idx == 1) ? ref_idx : -1;

                    cand_array[cand_idx].transform_type[0] = DCT_DCT;
                    cand_array[cand_idx].transform_type_uv = DCT_DCT;
                    if (list_idx == 0) {
                        cand_array[cand_idx].motion_vector_xl0 = to_inject_mv_x;
                        cand_array[cand_idx].motion_vector_yl0 = to_inject_mv_y;
                        context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] =
                            to_inject_mv_x;
                        context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] =
                            to_inject_mv_y;
                        context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] =
                            frame_type;
                        ++context_ptr->injected_mv_count_l0;
                    } else {
                        cand_array[cand_idx].motion_vector_xl1 = to_inject_mv_x;
                        cand_array[cand_idx].motion_vector_yl1 = to_inject_mv_y;
                        context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] =
                            to_inject_mv_x;
                        context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] =
                            to_inject_mv_y;
                        context_ptr->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] =
                            frame_type;
                        ++context_ptr->injected_mv_count_l1;
                    }
                    if (inter_type == 0) {
                        cand_array[cand_idx].is_interintra_used = 0;
                        cand_array[cand_idx].motion_mode        = SIMPLE_TRANSLATION;
                    } else {
                        if (is_ii_allowed) {
                            if (inter_type == 1) {
                                inter_intra_search(pcs_ptr, context_ptr, &cand_array[cand_idx]);
                                cand_array[cand_idx].is_interintra_used   = 1;
                                cand_array[cand_idx].use_wedge_interintra = 1;
                            } else if (inter_type == 2) {
                                cand_array[cand_idx].is_interintra_used = 1;
                                cand_array[cand_idx].interintra_mode =
                                    cand_array[cand_idx - 1].interintra_mode;
                                cand_array[cand_idx].use_wedge_interintra = 0;
                            }
                        }
                        if (is_obmc_allowed && inter_type == tot_inter_types - 1) {
                            cand_array[cand_idx].is_interintra_used = 0;
                            cand_array[cand_idx].motion_mode        = OBMC_CAUSAL;
                        }
                    }

                    INCRMENT_CAND_TOTAL_COUNT(cand_idx);
                }
            }
        }
    } else if (allow_compound) {
        uint8_t ref_idx_0 = get_ref_frame_idx(rf[0]);
        uint8_t ref_idx_1 = get_ref_frame_idx(rf[1]);
#if NEAREST_NEAR_REF_MASKING

        uint8_t list_idx_0 = get_list_idx(rf[0]);
        uint8_t list_idx_1 = get_list_idx(rf[1]);
        // Always consider the 2 closet ref frames (i.e. ref_idx=0) @ MVP cand generation
#if PRUNING_PER_INTER_TYPE
        if (!is_valid_bipred_ref(
            context_ptr, NRST_NEAR_GROUP, list_idx_0, ref_idx_0, list_idx_1, ref_idx_1)) return;
 #else
        if ((!context_ptr->ref_filtering_res[list_idx_0][ref_idx_0].do_ref || !context_ptr->ref_filtering_res[list_idx_1][ref_idx_1].do_ref) && (ref_idx_0 || ref_idx_1)) return;
#endif
#endif
        if (ref_idx_0 > context_ptr->md_max_ref_count - 1 ||
            ref_idx_1 > context_ptr->md_max_ref_count - 1)
            return;
        {
            //NEAREST_NEAREST
            int16_t to_inject_mv_x_l0 =
#if MEM_OPT_MV_STACK
                context_ptr->ed_ref_mv_stack[ref_pair][0].this_mv.as_mv.col;
#else
                context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                    .ed_ref_mv_stack[ref_pair][0]
                    .this_mv.as_mv.col;
#endif
            int16_t to_inject_mv_y_l0 =
#if MEM_OPT_MV_STACK
                context_ptr->ed_ref_mv_stack[ref_pair][0].this_mv.as_mv.row;
#else
                context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                    .ed_ref_mv_stack[ref_pair][0]
                    .this_mv.as_mv.row;
#endif
            int16_t to_inject_mv_x_l1 =
#if MEM_OPT_MV_STACK
                context_ptr->ed_ref_mv_stack[ref_pair][0].comp_mv.as_mv.col;
#else
                context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                    .ed_ref_mv_stack[ref_pair][0]
                    .comp_mv.as_mv.col;
#endif
            int16_t to_inject_mv_y_l1 =
#if MEM_OPT_MV_STACK
                context_ptr->ed_ref_mv_stack[ref_pair][0].comp_mv.as_mv.row;
#else
                context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                    .ed_ref_mv_stack[ref_pair][0]
                    .comp_mv.as_mv.row;
#endif

            inj_mv = context_ptr->injected_mv_count_bipred == 0 ||
                     mrp_is_already_injected_mv_bipred(context_ptr,
                                                       to_inject_mv_x_l0,
                                                       to_inject_mv_y_l0,
                                                       to_inject_mv_x_l1,
                                                       to_inject_mv_y_l1,
                                                       ref_pair) == EB_FALSE;

            if (umv0tile) {
                inside_tile = is_inside_tile_boundary(&(xd->tile),
                                                      to_inject_mv_x_l0,
                                                      to_inject_mv_y_l0,
                                                      mi_col,
                                                      mi_row,
                                                      context_ptr->blk_geom->bsize) &&
                              is_inside_tile_boundary(&(xd->tile),
                                                      to_inject_mv_x_l1,
                                                      to_inject_mv_y_l1,
                                                      mi_col,
                                                      mi_row,
                                                      context_ptr->blk_geom->bsize);
            }
            inj_mv = inj_mv && inside_tile;
            if (inj_mv) {
#if !INTER_COMP_REDESIGN
                context_ptr->variance_ready = 0;
#endif
#if INTER_COMP_REDESIGN
                if (ref_idx_0 > context_ptr->inter_comp_ctrls.mrp_pruning_w_distance - 1 &&
                    ref_idx_1 > context_ptr->inter_comp_ctrls.mrp_pruning_w_distance - 1)
                    tot_comp_types = MD_COMP_AVG;

#endif
                for (cur_type = MD_COMP_AVG; cur_type <= tot_comp_types; cur_type++) {
#if !INTER_COMP_REDESIGN
                    if (cur_type == MD_COMP_WEDGE &&
                            get_wedge_params_bits(context_ptr->blk_geom->bsize) == 0)
                        continue;
#endif
#if !REMOVE_SIMILARITY_FEATS
                    // If two predictors are very similar, skip wedge compound mode search
#if INTER_COMP_REDESIGN
                    if (cur_type == MD_COMP_WEDGE && context_ptr->inter_comp_ctrls.similar_predictions)
#else
                    if (context_ptr->variance_ready)
#endif
                        if (context_ptr->prediction_mse < 64)
                            continue;
#endif
                    cand_array[cand_idx].type               = INTER_MODE;
                    cand_array[cand_idx].inter_mode         = NEAREST_NEARESTMV;
                    cand_array[cand_idx].pred_mode          = NEAREST_NEARESTMV;
                    cand_array[cand_idx].motion_mode        = SIMPLE_TRANSLATION;
                    cand_array[cand_idx].is_compound        = 1;
                    cand_array[cand_idx].is_interintra_used = 0;
                    cand_array[cand_idx].distortion_ready   = 0;
                    cand_array[cand_idx].use_intrabc        = 0;
                    cand_array[cand_idx].merge_flag =
                        cur_type == MD_COMP_AVG && pcs_ptr->parent_pcs_ptr->is_skip_mode_allowed &&
                                (rf[0] == frm_hdr->skip_mode_params.ref_frame_idx_0 + 1) &&
                                (rf[1] == frm_hdr->skip_mode_params.ref_frame_idx_1 + 1)
                            ? EB_TRUE
                            : EB_FALSE;

                    cand_array[cand_idx].prediction_direction[0] = BI_PRED;
                    cand_array[cand_idx].is_new_mv               = 0;
                    cand_array[cand_idx].motion_vector_xl0       = to_inject_mv_x_l0;
                    cand_array[cand_idx].motion_vector_yl0       = to_inject_mv_y_l0;
                    cand_array[cand_idx].motion_vector_xl1       = to_inject_mv_x_l1;
                    cand_array[cand_idx].motion_vector_yl1       = to_inject_mv_y_l1;
                    cand_array[cand_idx].drl_index               = 0;
                    cand_array[cand_idx].ref_mv_index            = 0;
                    cand_array[cand_idx].ref_frame_type          = ref_pair;
                    cand_array[cand_idx].ref_frame_index_l0      = ref_idx_0;
                    cand_array[cand_idx].ref_frame_index_l1      = ref_idx_1;

                    cand_array[cand_idx].transform_type[0] = DCT_DCT;
                    cand_array[cand_idx].transform_type_uv = DCT_DCT;

                    context_ptr
                        ->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] =
                        to_inject_mv_x_l0;
                    context_ptr
                        ->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] =
                        to_inject_mv_y_l0;
                    context_ptr
                        ->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] =
                        to_inject_mv_x_l1;
                    context_ptr
                        ->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] =
                        to_inject_mv_y_l1;
                    context_ptr
                        ->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] =
                        ref_pair;
                    ++context_ptr->injected_mv_count_bipred;
                    //NRST-NRST
#if INTER_COMP_REDESIGN

                    if (cur_type == MD_COMP_AVG && tot_comp_types > MD_COMP_AVG)
                        calc_pred_masked_compound(
                            pcs_ptr, context_ptr, &cand_array[cand_idx]);
#if !REMOVE_SIMILARITY_FEATS
                    if (context_ptr->inter_comp_ctrls.similar_predictions)
                        if (cur_type > MD_COMP_AVG &&
                            context_ptr->prediction_mse <=
                            context_ptr->inter_comp_ctrls.similar_predictions_th )
                            continue;
#endif
#endif
                    determine_compound_mode(pcs_ptr, context_ptr, &cand_array[cand_idx], cur_type);
                    INCRMENT_CAND_TOTAL_COUNT(cand_idx);
                }
            }

            //NEAR_NEAR
            max_drl_index = get_max_drl_index(xd->ref_mv_count[ref_pair], NEAR_NEARMV);
            for (drli = 0; drli < max_drl_index; drli++) {
                get_av1_mv_pred_drl(context_ptr,
                                    blk_ptr,
                                    ref_pair,
                                    1,
                                    NEAR_NEARMV,
                                    drli,
                                    nearestmv,
                                    nearmv,
                                    ref_mv);

                to_inject_mv_x_l0 = nearmv[0].as_mv.col;
                to_inject_mv_y_l0 = nearmv[0].as_mv.row;
                to_inject_mv_x_l1 = nearmv[1].as_mv.col;
                to_inject_mv_y_l1 = nearmv[1].as_mv.row;

                inj_mv = context_ptr->injected_mv_count_bipred == 0 ||
                         mrp_is_already_injected_mv_bipred(context_ptr,
                                                           to_inject_mv_x_l0,
                                                           to_inject_mv_y_l0,
                                                           to_inject_mv_x_l1,
                                                           to_inject_mv_y_l1,
                                                           ref_pair) == EB_FALSE;

                if (umv0tile) {
                    inside_tile = is_inside_tile_boundary(&(xd->tile),
                                                          to_inject_mv_x_l0,
                                                          to_inject_mv_y_l0,
                                                          mi_col,
                                                          mi_row,
                                                          context_ptr->blk_geom->bsize) &&
                                  is_inside_tile_boundary(&(xd->tile),
                                                          to_inject_mv_x_l1,
                                                          to_inject_mv_y_l1,
                                                          mi_col,
                                                          mi_row,
                                                          context_ptr->blk_geom->bsize);
                }
                inj_mv = inj_mv && inside_tile;
                if (inj_mv) {
#if !INTER_COMP_REDESIGN
                    context_ptr->variance_ready = 0;
#endif
#if INTER_COMP_REDESIGN
                if (ref_idx_0 > context_ptr->inter_comp_ctrls.mrp_pruning_w_distance - 1 &&
                    ref_idx_1 > context_ptr->inter_comp_ctrls.mrp_pruning_w_distance - 1)
                    tot_comp_types = MD_COMP_AVG;
#endif
                    for (cur_type = MD_COMP_AVG; cur_type <= tot_comp_types; cur_type++) {
#if !INTER_COMP_REDESIGN
                        if (cur_type == MD_COMP_WEDGE &&
                                get_wedge_params_bits(context_ptr->blk_geom->bsize) == 0)
                            continue;
#endif
#if !REMOVE_SIMILARITY_FEATS
                        // If two predictors are very similar, skip wedge compound mode search
#if INTER_COMP_REDESIGN
                        if (cur_type == MD_COMP_WEDGE && context_ptr->inter_comp_ctrls.similar_predictions)
#else
                        if (context_ptr->variance_ready)
#endif
                            if (context_ptr->prediction_mse < 64)
                                continue;
#endif
                        cand_array[cand_idx].type                    = INTER_MODE;
                        cand_array[cand_idx].inter_mode              = NEAR_NEARMV;
                        cand_array[cand_idx].pred_mode               = NEAR_NEARMV;
                        cand_array[cand_idx].motion_mode             = SIMPLE_TRANSLATION;
                        cand_array[cand_idx].is_compound             = 1;
                        cand_array[cand_idx].is_interintra_used      = 0;
                        cand_array[cand_idx].distortion_ready        = 0;
                        cand_array[cand_idx].use_intrabc             = 0;
                        cand_array[cand_idx].merge_flag              = EB_FALSE;
                        cand_array[cand_idx].prediction_direction[0] = BI_PRED;
                        cand_array[cand_idx].is_new_mv               = 0;

                        cand_array[cand_idx].motion_vector_xl0 = to_inject_mv_x_l0;
                        cand_array[cand_idx].motion_vector_yl0 = to_inject_mv_y_l0;
                        cand_array[cand_idx].motion_vector_xl1 = to_inject_mv_x_l1;
                        cand_array[cand_idx].motion_vector_yl1 = to_inject_mv_y_l1;

                        cand_array[cand_idx].drl_index      = drli;
                        cand_array[cand_idx].ref_mv_index   = 0;
                        cand_array[cand_idx].ref_frame_type = ref_pair;

                        cand_array[cand_idx].ref_frame_index_l0 = ref_idx_0;
                        cand_array[cand_idx].ref_frame_index_l1 = ref_idx_1;

                        cand_array[cand_idx].transform_type[0] = DCT_DCT;
                        cand_array[cand_idx].transform_type_uv = DCT_DCT;

                        context_ptr
                            ->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_x_l0;
                        context_ptr
                            ->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_y_l0;
                        context_ptr
                            ->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_x_l1;
                        context_ptr
                            ->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_y_l1;
                        context_ptr->injected_ref_type_bipred_array
                            [context_ptr->injected_mv_count_bipred] = ref_pair;
                        ++context_ptr->injected_mv_count_bipred;
                        //NR-NR
#if INTER_COMP_REDESIGN

                        if (cur_type == MD_COMP_AVG && tot_comp_types > MD_COMP_AVG)
                            calc_pred_masked_compound(
                                pcs_ptr, context_ptr, &cand_array[cand_idx]);
#if !REMOVE_SIMILARITY_FEATS
                        if (context_ptr->inter_comp_ctrls.similar_predictions)
                            if (cur_type > MD_COMP_AVG &&
                                context_ptr->prediction_mse <=
                                context_ptr->inter_comp_ctrls.similar_predictions_th )
                                continue;
#endif
#endif
                        determine_compound_mode(
                            pcs_ptr, context_ptr, &cand_array[cand_idx], cur_type);
                        INCRMENT_CAND_TOTAL_COUNT(cand_idx);
                    }
                }
            }
        }
    }

    //update tot Candidate count
    *candTotCnt = cand_idx;
}

void inject_new_nearest_new_comb_candidates(const SequenceControlSet *  scs_ptr,
                                            struct ModeDecisionContext *context_ptr,
                                            PictureControlSet *pcs_ptr, MvReferenceFrame ref_pair,
                                            uint32_t *candTotCnt) {
    uint32_t               cand_idx   = *candTotCnt;
    ModeDecisionCandidate *cand_array = context_ptr->fast_candidate_array;
    MacroBlockD *          xd         = context_ptr->blk_ptr->av1xd;
    IntMv                  nearestmv[2], nearmv[2], ref_mv[2];
    int                    umv0tile    = (scs_ptr->static_config.unrestricted_motion_vector == 0);
    uint32_t               mi_row      = context_ptr->blk_origin_y >> MI_SIZE_LOG2;
    uint32_t               mi_col      = context_ptr->blk_origin_x >> MI_SIZE_LOG2;

    MvReferenceFrame rf[2];
    av1_set_ref_frame(rf, ref_pair);
#if !INTER_COMP_REDESIGN
    BlockSize    bsize          = context_ptr->blk_geom->bsize; // bloc size
#endif
#if INTER_COMP_REDESIGN
    MD_COMP_TYPE tot_comp_types = context_ptr->compound_types_to_try;
#else
    MD_COMP_TYPE tot_comp_types = (bsize >= BLOCK_8X8 && bsize <= BLOCK_32X32)
                                      ? context_ptr->compound_types_to_try
                                      : context_ptr->compound_types_to_try == MD_COMP_WEDGE
                                            ? MD_COMP_DIFF0
                                            : context_ptr->compound_types_to_try;

#endif
#if !INTER_COMP_REDESIGN

    if (context_ptr->source_variance < context_ptr->inter_inter_wedge_variance_th)
        tot_comp_types = MIN(tot_comp_types, MD_COMP_DIFF0);

#else
#if !OPT_4
    if (context_ptr->source_variance < context_ptr->inter_comp_ctrls.wedge_variance_th)
        tot_comp_types = MIN(tot_comp_types, MD_COMP_DIFF0);
#endif
#endif
    {
        uint8_t ref_idx_0 = get_ref_frame_idx(rf[0]);
        uint8_t ref_idx_1 = get_ref_frame_idx(rf[1]);
#if NEW_NEAREST_NEW_NEAR_REF_MASKING
        uint8_t list_idx_0 = get_list_idx(rf[0]);
        uint8_t list_idx_1 = get_list_idx(rf[1]);
        if (list_idx_0 != INVALID_REF)
#if PRUNING_PER_INTER_TYPE
            if (!is_valid_unipred_ref(context_ptr, MIN(TOT_INTER_GROUP-1,NRST_NEW_NEAR_GROUP), list_idx_0, ref_idx_0)) return;
#else
            if (!context_ptr->ref_filtering_res[list_idx_0][ref_idx_0].do_ref) return;
#endif
        if (list_idx_1 != INVALID_REF)
#if PRUNING_PER_INTER_TYPE
            if (!is_valid_unipred_ref(context_ptr, MIN(TOT_INTER_GROUP-1,NRST_NEW_NEAR_GROUP), list_idx_1, ref_idx_1)) return;
#else
            if (!context_ptr->ref_filtering_res[list_idx_1][ref_idx_1].do_ref) return;
#endif
#endif
        if (ref_idx_0 > context_ptr->md_max_ref_count - 1 ||
            ref_idx_1 > context_ptr->md_max_ref_count - 1)
            return;

        if (rf[1] != NONE_FRAME) {
            {
                //NEAREST_NEWMV
#if DECOUPLE_ME_RES
                const MeSbResults *me_results =
                    pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[context_ptr->me_sb_addr];
#else
                const MeSbResults *me_results =
                    pcs_ptr->parent_pcs_ptr->me_results[context_ptr->me_sb_addr];
#endif

                int16_t to_inject_mv_x_l0 =
#if MEM_OPT_MV_STACK
                    context_ptr->ed_ref_mv_stack[ref_pair][0].this_mv.as_mv.col;
#else
                    context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                        .ed_ref_mv_stack[ref_pair][0]
                        .this_mv.as_mv.col;
#endif
                int16_t to_inject_mv_y_l0 =
#if MEM_OPT_MV_STACK
                    context_ptr->ed_ref_mv_stack[ref_pair][0].this_mv.as_mv.row;
#else
                    context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                        .ed_ref_mv_stack[ref_pair][0]
                        .this_mv.as_mv.row;
#endif
                int16_t to_inject_mv_x_l1 =
                    context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds][get_list_idx(rf[1])]
                    [ref_idx_1][0];
                int16_t to_inject_mv_y_l1 =
                    context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds][get_list_idx(rf[1])]
                    [ref_idx_1][1];
                uint8_t inj_mv = context_ptr->injected_mv_count_bipred == 0 ||
                         mrp_is_already_injected_mv_bipred(context_ptr,
                                                           to_inject_mv_x_l0,
                                                           to_inject_mv_y_l0,
                                                           to_inject_mv_x_l1,
                                                           to_inject_mv_y_l1,
                                                           ref_pair) == EB_FALSE;

                int inside_tile = umv0tile ? is_inside_tile_boundary(&(xd->tile),
                                                          to_inject_mv_x_l0,
                                                          to_inject_mv_y_l0,
                                                          mi_col,
                                                          mi_row,
                                                          context_ptr->blk_geom->bsize) &&
                                  is_inside_tile_boundary(&(xd->tile),
                                                          to_inject_mv_x_l1,
                                                          to_inject_mv_y_l1,
                                                          mi_col,
                                                          mi_row,
                                                          context_ptr->blk_geom->bsize) : 1;
                inj_mv = inj_mv && inside_tile;
                inj_mv = inj_mv && is_me_data_present(context_ptr, me_results, get_list_idx(rf[1]), ref_idx_1);
                if (inj_mv) {
#if !INTER_COMP_REDESIGN
                    context_ptr->variance_ready = 0;
#endif
#if INTER_COMP_REDESIGN
                if (ref_idx_0 > context_ptr->inter_comp_ctrls.mrp_pruning_w_distance - 1 &&
                    ref_idx_1 > context_ptr->inter_comp_ctrls.mrp_pruning_w_distance - 1)
                    tot_comp_types = MD_COMP_AVG;
#endif
                    for (MD_COMP_TYPE cur_type = MD_COMP_AVG; cur_type <= tot_comp_types; cur_type++) {
#if !INTER_COMP_REDESIGN
                        if (cur_type == MD_COMP_WEDGE &&
                                get_wedge_params_bits(context_ptr->blk_geom->bsize) == 0)
                            continue;
#endif
#if !REMOVE_SIMILARITY_FEATS
                        // If two predictors are very similar, skip wedge compound mode search
#if INTER_COMP_REDESIGN
                        if (cur_type == MD_COMP_WEDGE && context_ptr->inter_comp_ctrls.similar_predictions)
#else
                        if (context_ptr->variance_ready)
#endif
                            if (context_ptr->prediction_mse < 8)
                                continue;
#endif
                        cand_array[cand_idx].type               = INTER_MODE;
                        cand_array[cand_idx].inter_mode         = NEAREST_NEWMV;
                        cand_array[cand_idx].pred_mode          = NEAREST_NEWMV;
                        cand_array[cand_idx].motion_mode        = SIMPLE_TRANSLATION;
                        cand_array[cand_idx].is_compound        = 1;
                        cand_array[cand_idx].is_interintra_used = 0;
                        cand_array[cand_idx].distortion_ready   = 0;
                        cand_array[cand_idx].use_intrabc        = 0;

                        cand_array[cand_idx].merge_flag = EB_FALSE;

                        cand_array[cand_idx].prediction_direction[0] = BI_PRED;
                        cand_array[cand_idx].is_new_mv               = 0;
                        cand_array[cand_idx].motion_vector_xl0       = to_inject_mv_x_l0;
                        cand_array[cand_idx].motion_vector_yl0       = to_inject_mv_y_l0;
                        cand_array[cand_idx].motion_vector_xl1       = to_inject_mv_x_l1;
                        cand_array[cand_idx].motion_vector_yl1       = to_inject_mv_y_l1;
                        cand_array[cand_idx].drl_index               = 0;
                        cand_array[cand_idx].ref_mv_index            = 0;
                        cand_array[cand_idx].ref_frame_type          = ref_pair;
                        cand_array[cand_idx].ref_frame_index_l0      = ref_idx_0;
                        cand_array[cand_idx].ref_frame_index_l1      = ref_idx_1;
                        cand_array[cand_idx].transform_type[0]       = DCT_DCT;
                        cand_array[cand_idx].transform_type_uv       = DCT_DCT;
                        get_av1_mv_pred_drl(context_ptr,
                                            context_ptr->blk_ptr,
                                            cand_array[cand_idx].ref_frame_type,
                                            cand_array[cand_idx].is_compound,
                                            NEAREST_NEWMV,
                                            0, //not needed drli,
                                            nearestmv,
                                            nearmv,
                                            ref_mv);
                        cand_array[cand_idx].motion_vector_pred_x[REF_LIST_1] = ref_mv[1].as_mv.col;
                        cand_array[cand_idx].motion_vector_pred_y[REF_LIST_1] = ref_mv[1].as_mv.row;
                        context_ptr
                            ->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_x_l0;
                        context_ptr
                            ->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_y_l0;
                        context_ptr
                            ->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_x_l1;
                        context_ptr
                            ->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_y_l1;
                        context_ptr->injected_ref_type_bipred_array
                            [context_ptr->injected_mv_count_bipred] = ref_pair;
                        ++context_ptr->injected_mv_count_bipred;
                        //NRST_N
#if INTER_COMP_REDESIGN
                        if (cur_type == MD_COMP_AVG && tot_comp_types > MD_COMP_AVG)
                            calc_pred_masked_compound(
                                pcs_ptr, context_ptr, &cand_array[cand_idx]);
#if !REMOVE_SIMILARITY_FEATS
                        if (context_ptr->inter_comp_ctrls.similar_predictions)
                            if (cur_type > MD_COMP_AVG &&
                                context_ptr->prediction_mse <=
                                context_ptr->inter_comp_ctrls.similar_predictions_th )
                                continue;
#endif
#endif
                        determine_compound_mode(
                            pcs_ptr, context_ptr, &cand_array[cand_idx], cur_type);
                        INCRMENT_CAND_TOTAL_COUNT(cand_idx);
                    }
                }
            }

            {
                //NEW_NEARESTMV
#if DECOUPLE_ME_RES
                const MeSbResults *me_results =
                    pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[context_ptr->me_sb_addr];
#else
                const MeSbResults *me_results =
                    pcs_ptr->parent_pcs_ptr->me_results[context_ptr->me_sb_addr];
#endif
                int16_t to_inject_mv_x_l0 =
                    context_ptr
                    ->sb_me_mv[context_ptr->blk_geom->blkidx_mds][REF_LIST_0][ref_idx_0][0];
                int16_t to_inject_mv_y_l0 =
                    context_ptr
                    ->sb_me_mv[context_ptr->blk_geom->blkidx_mds][REF_LIST_0][ref_idx_0][1];
                int16_t to_inject_mv_x_l1 =
#if MEM_OPT_MV_STACK
                    context_ptr->ed_ref_mv_stack[ref_pair][0].comp_mv.as_mv.col;
#else
                    context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                        .ed_ref_mv_stack[ref_pair][0]
                        .comp_mv.as_mv.col;
#endif
                int16_t to_inject_mv_y_l1 =
#if MEM_OPT_MV_STACK
                    context_ptr->ed_ref_mv_stack[ref_pair][0].comp_mv.as_mv.row;
#else
                    context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                        .ed_ref_mv_stack[ref_pair][0]
                        .comp_mv.as_mv.row;
#endif

                uint8_t inj_mv = context_ptr->injected_mv_count_bipred == 0 ||
                         mrp_is_already_injected_mv_bipred(context_ptr,
                                                           to_inject_mv_x_l0,
                                                           to_inject_mv_y_l0,
                                                           to_inject_mv_x_l1,
                                                           to_inject_mv_y_l1,
                                                           ref_pair) == EB_FALSE;

                int inside_tile = umv0tile ? is_inside_tile_boundary(&(xd->tile),
                                                          to_inject_mv_x_l0,
                                                          to_inject_mv_y_l0,
                                                          mi_col,
                                                          mi_row,
                                                          context_ptr->blk_geom->bsize) &&
                                  is_inside_tile_boundary(&(xd->tile),
                                                          to_inject_mv_x_l1,
                                                          to_inject_mv_y_l1,
                                                          mi_col,
                                                          mi_row,
                                                          context_ptr->blk_geom->bsize) : 1;
                inj_mv = inj_mv && inside_tile;
                inj_mv = inj_mv && is_me_data_present(context_ptr, me_results, 0, ref_idx_0);
                if (inj_mv) {
#if !INTER_COMP_REDESIGN
                    context_ptr->variance_ready = 0;
#endif
#if INTER_COMP_REDESIGN
                    if (ref_idx_0 > context_ptr->inter_comp_ctrls.mrp_pruning_w_distance - 1 &&
                        ref_idx_1 > context_ptr->inter_comp_ctrls.mrp_pruning_w_distance - 1)
                        tot_comp_types = MD_COMP_AVG;
#endif
                    for (MD_COMP_TYPE cur_type = MD_COMP_AVG; cur_type <= tot_comp_types; cur_type++) {
#if !INTER_COMP_REDESIGN
                        if (cur_type == MD_COMP_WEDGE &&
                                get_wedge_params_bits(context_ptr->blk_geom->bsize) == 0)
                            continue;
#endif
#if !REMOVE_SIMILARITY_FEATS
                        // If two predictors are very similar, skip wedge compound mode search
#if INTER_COMP_REDESIGN
                        if (cur_type == MD_COMP_WEDGE && context_ptr->inter_comp_ctrls.similar_predictions)
#else
                        if (context_ptr->variance_ready)
#endif
                            if (context_ptr->prediction_mse < 8)
                                continue;
#endif
                        cand_array[cand_idx].type                    = INTER_MODE;
                        cand_array[cand_idx].inter_mode              = NEW_NEARESTMV;
                        cand_array[cand_idx].pred_mode               = NEW_NEARESTMV;
                        cand_array[cand_idx].motion_mode             = SIMPLE_TRANSLATION;
                        cand_array[cand_idx].is_compound             = 1;
                        cand_array[cand_idx].is_interintra_used      = 0;
                        cand_array[cand_idx].distortion_ready        = 0;
                        cand_array[cand_idx].use_intrabc             = 0;
                        cand_array[cand_idx].merge_flag              = EB_FALSE;
                        cand_array[cand_idx].prediction_direction[0] = BI_PRED;
                        cand_array[cand_idx].is_new_mv               = 0;
                        cand_array[cand_idx].motion_vector_xl0       = to_inject_mv_x_l0;
                        cand_array[cand_idx].motion_vector_yl0       = to_inject_mv_y_l0;
                        cand_array[cand_idx].motion_vector_xl1       = to_inject_mv_x_l1;
                        cand_array[cand_idx].motion_vector_yl1       = to_inject_mv_y_l1;
                        cand_array[cand_idx].drl_index               = 0;
                        cand_array[cand_idx].ref_mv_index            = 0;
                        cand_array[cand_idx].ref_frame_type          = ref_pair;
                        cand_array[cand_idx].ref_frame_index_l0      = ref_idx_0;
                        cand_array[cand_idx].ref_frame_index_l1      = ref_idx_1;
                        cand_array[cand_idx].transform_type[0]       = DCT_DCT;
                        cand_array[cand_idx].transform_type_uv       = DCT_DCT;
                        get_av1_mv_pred_drl(context_ptr,
                                            context_ptr->blk_ptr,
                                            cand_array[cand_idx].ref_frame_type,
                                            cand_array[cand_idx].is_compound,
                                            NEW_NEARESTMV,
                                            0, //not needed drli,
                                            nearestmv,
                                            nearmv,
                                            ref_mv);
                        cand_array[cand_idx].motion_vector_pred_x[REF_LIST_0] = ref_mv[0].as_mv.col;
                        cand_array[cand_idx].motion_vector_pred_y[REF_LIST_0] = ref_mv[0].as_mv.row;
                        context_ptr
                            ->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_x_l0;
                        context_ptr
                            ->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_y_l0;
                        context_ptr
                            ->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_x_l1;
                        context_ptr
                            ->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_y_l1;
                        context_ptr->injected_ref_type_bipred_array
                            [context_ptr->injected_mv_count_bipred] = ref_pair;
                        ++context_ptr->injected_mv_count_bipred;
                        //N_NRST
#if INTER_COMP_REDESIGN

                        if (cur_type == MD_COMP_AVG && tot_comp_types > MD_COMP_AVG)
                            calc_pred_masked_compound(
                                pcs_ptr, context_ptr, &cand_array[cand_idx]);
#if !REMOVE_SIMILARITY_FEATS
                        if (context_ptr->inter_comp_ctrls.similar_predictions)
                            if (cur_type > MD_COMP_AVG &&
                                context_ptr->prediction_mse <=
                                context_ptr->inter_comp_ctrls.similar_predictions_th )
                                continue;
#endif
#endif
                        determine_compound_mode(
                            pcs_ptr, context_ptr, &cand_array[cand_idx], cur_type);
                        INCRMENT_CAND_TOTAL_COUNT(cand_idx);
                    }
                }
            }
            //NEW_NEARMV
            {
                uint8_t max_drl_index = get_max_drl_index(xd->ref_mv_count[ref_pair], NEW_NEARMV);

                for (uint8_t drli = 0; drli < max_drl_index; drli++) {
                    get_av1_mv_pred_drl(context_ptr,
                                        context_ptr->blk_ptr,
                                        ref_pair,
                                        1,
                                        NEW_NEARMV,
                                        drli,
                                        nearestmv,
                                        nearmv,
                                        ref_mv);

                    //NEW_NEARMV
#if DECOUPLE_ME_RES
                    const MeSbResults *me_results =
                        pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[context_ptr->me_sb_addr];
#else
                    const MeSbResults *me_results =
                        pcs_ptr->parent_pcs_ptr->me_results[context_ptr->me_sb_addr];
#endif
                    int16_t to_inject_mv_x_l0 =
                        context_ptr
                        ->sb_me_mv[context_ptr->blk_geom->blkidx_mds][REF_LIST_0][ref_idx_0][0];
                    int16_t to_inject_mv_y_l0 =
                        context_ptr
                        ->sb_me_mv[context_ptr->blk_geom->blkidx_mds][REF_LIST_0][ref_idx_0][1];
                    int16_t to_inject_mv_x_l1 = nearmv[1].as_mv.col;
                    int16_t to_inject_mv_y_l1 = nearmv[1].as_mv.row;

                    uint8_t inj_mv = context_ptr->injected_mv_count_bipred == 0 ||
                             mrp_is_already_injected_mv_bipred(context_ptr,
                                                               to_inject_mv_x_l0,
                                                               to_inject_mv_y_l0,
                                                               to_inject_mv_x_l1,
                                                               to_inject_mv_y_l1,
                                                               ref_pair) == EB_FALSE;
                    inj_mv = inj_mv && is_me_data_present(context_ptr, me_results, 0, ref_idx_0);
                    if (inj_mv) {
#if !INTER_COMP_REDESIGN
                        context_ptr->variance_ready = 0;
#endif
#if INTER_COMP_REDESIGN
                if (ref_idx_0 > context_ptr->inter_comp_ctrls.mrp_pruning_w_distance - 1 &&
                    ref_idx_1 > context_ptr->inter_comp_ctrls.mrp_pruning_w_distance - 1)
                    tot_comp_types = MD_COMP_AVG;

#endif
                        for (MD_COMP_TYPE cur_type = MD_COMP_AVG; cur_type <= tot_comp_types; cur_type++) {
#if !REMOVE_SIMILARITY_FEATS
                            // If two predictors are very similar, skip wedge compound mode search
#if INTER_COMP_REDESIGN
                            if (cur_type == MD_COMP_WEDGE && context_ptr->inter_comp_ctrls.similar_predictions)
#else
                            if (context_ptr->variance_ready)
#endif
                                if (context_ptr->prediction_mse < 8)
                                    continue;
#endif
                            cand_array[cand_idx].type               = INTER_MODE;
                            cand_array[cand_idx].inter_mode         = NEW_NEARMV;
                            cand_array[cand_idx].pred_mode          = NEW_NEARMV;
                            cand_array[cand_idx].motion_mode        = SIMPLE_TRANSLATION;
                            cand_array[cand_idx].is_compound        = 1;
                            cand_array[cand_idx].is_interintra_used = 0;

                            cand_array[cand_idx].distortion_ready = 0;
                            cand_array[cand_idx].use_intrabc      = 0;
                            cand_array[cand_idx].merge_flag       = EB_FALSE;

                            cand_array[cand_idx].prediction_direction[0] = BI_PRED;
                            cand_array[cand_idx].is_new_mv               = 0;
                            cand_array[cand_idx].motion_vector_xl0       = to_inject_mv_x_l0;
                            cand_array[cand_idx].motion_vector_yl0       = to_inject_mv_y_l0;
                            cand_array[cand_idx].motion_vector_xl1       = to_inject_mv_x_l1;
                            cand_array[cand_idx].motion_vector_yl1       = to_inject_mv_y_l1;

                            cand_array[cand_idx].drl_index = drli;

                            cand_array[cand_idx].ref_mv_index       = 0;
                            cand_array[cand_idx].ref_frame_type     = ref_pair;
                            cand_array[cand_idx].ref_frame_index_l0 = ref_idx_0;
                            cand_array[cand_idx].ref_frame_index_l1 = ref_idx_1;

                            cand_array[cand_idx].transform_type[0] = DCT_DCT;
                            cand_array[cand_idx].transform_type_uv = DCT_DCT;

                            cand_array[cand_idx].motion_vector_pred_x[REF_LIST_0] =
                                ref_mv[0].as_mv.col;
                            cand_array[cand_idx].motion_vector_pred_y[REF_LIST_0] =
                                ref_mv[0].as_mv.row;

                            context_ptr->injected_mv_x_bipred_l0_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                            context_ptr->injected_mv_y_bipred_l0_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                            context_ptr->injected_mv_x_bipred_l1_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                            context_ptr->injected_mv_y_bipred_l1_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                            context_ptr->injected_ref_type_bipred_array
                                [context_ptr->injected_mv_count_bipred] = ref_pair;
                            ++context_ptr->injected_mv_count_bipred;

                            //NEW_NEARMV
#if INTER_COMP_REDESIGN
                            if (cur_type == MD_COMP_AVG && tot_comp_types > MD_COMP_AVG)
                                calc_pred_masked_compound(
                                    pcs_ptr, context_ptr, &cand_array[cand_idx]);
#if !REMOVE_SIMILARITY_FEATS
                            if (context_ptr->inter_comp_ctrls.similar_predictions)
                                if (cur_type > MD_COMP_AVG &&
                                    context_ptr->prediction_mse <=
                                    context_ptr->inter_comp_ctrls.similar_predictions_th )
                                    continue;
#endif
#endif
                            determine_compound_mode(
                                pcs_ptr, context_ptr, &cand_array[cand_idx], cur_type);

                            INCRMENT_CAND_TOTAL_COUNT(cand_idx);
                        }
                    }
                }
            }
            //NEAR_NEWMV
            {
                uint8_t max_drl_index = get_max_drl_index(xd->ref_mv_count[ref_pair], NEAR_NEWMV);

                for (uint8_t drli = 0; drli < max_drl_index; drli++) {
                    get_av1_mv_pred_drl(context_ptr,
                                        context_ptr->blk_ptr,
                                        ref_pair,
                                        1,
                                        NEAR_NEWMV,
                                        drli,
                                        nearestmv,
                                        nearmv,
                                        ref_mv);

                    //NEAR_NEWMV
#if DECOUPLE_ME_RES
                    const MeSbResults *me_results =
                        pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[context_ptr->me_sb_addr];
#else
                    const MeSbResults *me_results =
                        pcs_ptr->parent_pcs_ptr->me_results[context_ptr->me_sb_addr];
#endif

                    int16_t to_inject_mv_x_l0 = nearmv[0].as_mv.col;
                    int16_t to_inject_mv_y_l0 = nearmv[0].as_mv.row;
                    int16_t to_inject_mv_x_l1 =
                        context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                        [get_list_idx(rf[1])][ref_idx_1][0];
                    int16_t to_inject_mv_y_l1 =
                        context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                        [get_list_idx(rf[1])][ref_idx_1][1];
                    uint8_t inj_mv = context_ptr->injected_mv_count_bipred == 0 ||
                             mrp_is_already_injected_mv_bipred(context_ptr,
                                                               to_inject_mv_x_l0,
                                                               to_inject_mv_y_l0,
                                                               to_inject_mv_x_l1,
                                                               to_inject_mv_y_l1,
                                                               ref_pair) == EB_FALSE;
                    inj_mv = inj_mv && is_me_data_present(context_ptr, me_results, get_list_idx(rf[1]), ref_idx_1);
                    if (inj_mv) {
#if !INTER_COMP_REDESIGN
                        context_ptr->variance_ready = 0;
#endif
#if INTER_COMP_REDESIGN
                    if (ref_idx_0 > context_ptr->inter_comp_ctrls.mrp_pruning_w_distance - 1 &&
                        ref_idx_1 > context_ptr->inter_comp_ctrls.mrp_pruning_w_distance - 1)
                        tot_comp_types = MD_COMP_AVG;

#endif
                        for (MD_COMP_TYPE cur_type = MD_COMP_AVG; cur_type <= tot_comp_types; cur_type++) {
#if !REMOVE_SIMILARITY_FEATS
                            // If two predictors are very similar, skip wedge compound mode search
#if INTER_COMP_REDESIGN
                            if (cur_type == MD_COMP_WEDGE && context_ptr->inter_comp_ctrls.similar_predictions)
#else
                            if (context_ptr->variance_ready)
#endif
                                if (context_ptr->prediction_mse < 8)
                                    continue;
#endif
                            cand_array[cand_idx].type               = INTER_MODE;
                            cand_array[cand_idx].inter_mode         = NEAR_NEWMV;
                            cand_array[cand_idx].pred_mode          = NEAR_NEWMV;
                            cand_array[cand_idx].motion_mode        = SIMPLE_TRANSLATION;
                            cand_array[cand_idx].is_compound        = 1;
                            cand_array[cand_idx].is_interintra_used = 0;

                            cand_array[cand_idx].distortion_ready = 0;
                            cand_array[cand_idx].use_intrabc      = 0;
                            cand_array[cand_idx].merge_flag       = EB_FALSE;

                            cand_array[cand_idx].prediction_direction[0] = BI_PRED;
                            cand_array[cand_idx].is_new_mv               = 0;
                            cand_array[cand_idx].motion_vector_xl0       = to_inject_mv_x_l0;
                            cand_array[cand_idx].motion_vector_yl0       = to_inject_mv_y_l0;
                            cand_array[cand_idx].motion_vector_xl1       = to_inject_mv_x_l1;
                            cand_array[cand_idx].motion_vector_yl1       = to_inject_mv_y_l1;
                            cand_array[cand_idx].drl_index               = drli;
                            cand_array[cand_idx].ref_mv_index            = 0;
                            cand_array[cand_idx].ref_frame_type          = ref_pair;
                            cand_array[cand_idx].ref_frame_index_l0      = ref_idx_0;
                            cand_array[cand_idx].ref_frame_index_l1      = ref_idx_1;

                            cand_array[cand_idx].transform_type[0] = DCT_DCT;
                            cand_array[cand_idx].transform_type_uv = DCT_DCT;

                            cand_array[cand_idx].motion_vector_pred_x[REF_LIST_1] =
                                ref_mv[1].as_mv.col;
                            cand_array[cand_idx].motion_vector_pred_y[REF_LIST_1] =
                                ref_mv[1].as_mv.row;

                            context_ptr->injected_mv_x_bipred_l0_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                            context_ptr->injected_mv_y_bipred_l0_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                            context_ptr->injected_mv_x_bipred_l1_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                            context_ptr->injected_mv_y_bipred_l1_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                            context_ptr->injected_ref_type_bipred_array
                                [context_ptr->injected_mv_count_bipred] = ref_pair;
                            ++context_ptr->injected_mv_count_bipred;

                            //NEAR_NEWMV
#if INTER_COMP_REDESIGN
                            if (cur_type == MD_COMP_AVG && tot_comp_types > MD_COMP_AVG)
                                calc_pred_masked_compound(
                                    pcs_ptr, context_ptr, &cand_array[cand_idx]);
#if !REMOVE_SIMILARITY_FEATS
                            if (context_ptr->inter_comp_ctrls.similar_predictions)
                                if (cur_type > MD_COMP_AVG &&
                                    context_ptr->prediction_mse <=
                                    context_ptr->inter_comp_ctrls.similar_predictions_th )
                                    continue;
#endif
#endif
                            determine_compound_mode(
                                pcs_ptr, context_ptr, &cand_array[cand_idx], cur_type);

                            INCRMENT_CAND_TOTAL_COUNT(cand_idx);
                        }
                    }
                }
            }
        }
    }

    //update tot Candidate count
    *candTotCnt = cand_idx;
}
void inject_warped_motion_candidates(
    PictureControlSet              *pcs_ptr,
    struct ModeDecisionContext     *context_ptr,
    BlkStruct                      *blk_ptr,
    uint32_t                       *cand_tot_cnt,
    MeSbResults                    *me_results) {
    uint32_t can_idx = *cand_tot_cnt;
    ModeDecisionCandidate *cand_array = context_ptr->fast_candidate_array;
    MacroBlockD  *xd = blk_ptr->av1xd;
    uint8_t drli, max_drl_index;
    IntMv nearest_mv[2], near_mv[2], ref_mv[2];

    int inside_tile = 1;
    SequenceControlSet *scs_ptr =
        (SequenceControlSet *)pcs_ptr->parent_pcs_ptr->scs_wrapper_ptr->object_ptr;
    int umv0_tile = (scs_ptr->static_config.unrestricted_motion_vector == 0);
    uint32_t mi_row = context_ptr->blk_origin_y >> MI_SIZE_LOG2;
    uint32_t mi_col = context_ptr->blk_origin_x >> MI_SIZE_LOG2;
    uint32_t ref_it;
    MvReferenceFrame rf[2];
    Mv mv_0;
    MvUnit mv_unit;
    int16_t to_inject_mv_x, to_inject_mv_y;
    //all of ref pairs: (1)single-ref List0  (2)single-ref List1
    for (ref_it = 0; ref_it < pcs_ptr->parent_pcs_ptr->tot_ref_frame_types; ++ref_it) {
        MvReferenceFrame ref_frame_pair = pcs_ptr->parent_pcs_ptr->ref_frame_type_arr[ref_it];
        av1_set_ref_frame(rf, ref_frame_pair);

        //single ref/list
        if (rf[1] == NONE_FRAME)
        {
            MvReferenceFrame frame_type = rf[0];
            uint8_t list_idx = get_list_idx(rf[0]);
            uint8_t ref_idx = get_ref_frame_idx(rf[0]);
#if WARP_REF_MASKING
#if PRUNING_PER_INTER_TYPE
            if (!is_valid_unipred_ref(context_ptr, MIN(TOT_INTER_GROUP-1,WARP_GROUP), list_idx, ref_idx)) continue;
#else
            if (!context_ptr->ref_filtering_res[list_idx][ref_idx].do_ref) continue;
#endif
#endif
            if (ref_idx > context_ptr->md_max_ref_count - 1)
                continue;
            //NEAREST
            to_inject_mv_x = context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                                 .ref_mvs[frame_type][0]
                                 .as_mv.col;
            to_inject_mv_y = context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                                 .ref_mvs[frame_type][0]
                                 .as_mv.row;

            if (umv0_tile)
                inside_tile = is_inside_tile_boundary(&(xd->tile), to_inject_mv_x, to_inject_mv_y, mi_col, mi_row, context_ptr->blk_geom->bsize);
            if (inside_tile)
            {
                cand_array[can_idx].type = INTER_MODE;
                cand_array[can_idx].inter_mode = NEARESTMV;
                cand_array[can_idx].pred_mode = NEARESTMV;
                cand_array[can_idx].motion_mode = WARPED_CAUSAL;
                cand_array[can_idx].wm_params_l0.wmtype = AFFINE;
                cand_array[can_idx].is_compound = 0;
                cand_array[can_idx].is_interintra_used = 0;
                cand_array[can_idx].distortion_ready = 0;
                cand_array[can_idx].use_intrabc = 0;
                cand_array[can_idx].merge_flag = EB_FALSE;
                cand_array[can_idx].prediction_direction[0] = list_idx;
                cand_array[can_idx].is_new_mv = 0;
                if (list_idx == 0) {
                    cand_array[can_idx].motion_vector_xl0 = to_inject_mv_x;
                    cand_array[can_idx].motion_vector_yl0 = to_inject_mv_y;
                }
                else {
                    cand_array[can_idx].motion_vector_xl1 = to_inject_mv_x;
                    cand_array[can_idx].motion_vector_yl1 = to_inject_mv_y;
                }
                cand_array[can_idx].drl_index = 0;
                cand_array[can_idx].ref_mv_index = 0;
                cand_array[can_idx].ref_frame_type = frame_type;
                cand_array[can_idx].ref_frame_index_l0 = (list_idx == 0) ? ref_idx : -1;
                cand_array[can_idx].ref_frame_index_l1 = (list_idx == 1) ? ref_idx : -1;
                cand_array[can_idx].transform_type[0] = DCT_DCT;
                cand_array[can_idx].transform_type_uv = DCT_DCT;
                mv_0.x = to_inject_mv_x;
                mv_0.y = to_inject_mv_y;
                mv_unit.mv[list_idx] = mv_0;
                mv_unit.pred_direction = cand_array[can_idx].prediction_direction[0];
                cand_array[can_idx].local_warp_valid = warped_motion_parameters(
                    pcs_ptr,
                    context_ptr->blk_ptr,
                    &mv_unit,
                    context_ptr->blk_geom,
                    context_ptr->blk_origin_x,
                    context_ptr->blk_origin_y,
                    cand_array[can_idx].ref_frame_type,
                    &cand_array[can_idx].wm_params_l0,
                    &cand_array[can_idx].num_proj_ref);

                if (cand_array[can_idx].local_warp_valid)
                    INCRMENT_CAND_TOTAL_COUNT(can_idx);
            }
            //NEAR
            max_drl_index = get_max_drl_index(xd->ref_mv_count[frame_type], NEARMV);
            for (drli = 0; drli < max_drl_index; drli++) {
                get_av1_mv_pred_drl(
                    context_ptr,
                    blk_ptr,
                    frame_type,
                    0,
                    NEARMV,
                    drli,
                    nearest_mv,
                    near_mv,
                    ref_mv);

                to_inject_mv_x = near_mv[0].as_mv.col;
                to_inject_mv_y = near_mv[0].as_mv.row;

                if (umv0_tile)
                    inside_tile = is_inside_tile_boundary(&(xd->tile), to_inject_mv_x, to_inject_mv_y, mi_col, mi_row, context_ptr->blk_geom->bsize);
                if (inside_tile)
                {
                    cand_array[can_idx].type = INTER_MODE;
                    cand_array[can_idx].inter_mode = NEARMV;
                    cand_array[can_idx].pred_mode = NEARMV;
                    cand_array[can_idx].motion_mode = WARPED_CAUSAL;
                    cand_array[can_idx].wm_params_l0.wmtype = AFFINE;
                    cand_array[can_idx].is_compound = 0;
                    cand_array[can_idx].is_interintra_used = 0;
                    cand_array[can_idx].distortion_ready = 0;
                    cand_array[can_idx].use_intrabc = 0;
                    cand_array[can_idx].merge_flag = EB_FALSE;
                    cand_array[can_idx].prediction_direction[0] = list_idx;
                    cand_array[can_idx].is_new_mv = 0;
                    if (list_idx == 0) {
                        cand_array[can_idx].motion_vector_xl0 = to_inject_mv_x;
                        cand_array[can_idx].motion_vector_yl0 = to_inject_mv_y;
                    }
                    else {
                        cand_array[can_idx].motion_vector_xl1 = to_inject_mv_x;
                        cand_array[can_idx].motion_vector_yl1 = to_inject_mv_y;
                    }
                    cand_array[can_idx].drl_index = drli;
                    cand_array[can_idx].ref_mv_index = 0;
                    cand_array[can_idx].ref_frame_type = frame_type;
                    cand_array[can_idx].ref_frame_index_l0 = (list_idx == 0) ? ref_idx : -1;
                    cand_array[can_idx].ref_frame_index_l1 = (list_idx == 1) ? ref_idx : -1;
                    cand_array[can_idx].transform_type[0] = DCT_DCT;
                    cand_array[can_idx].transform_type_uv = DCT_DCT;
                    mv_0.x = to_inject_mv_x;
                    mv_0.y = to_inject_mv_y;
                    mv_unit.mv[list_idx] = mv_0;
                    mv_unit.pred_direction = cand_array[can_idx].prediction_direction[0];

                    cand_array[can_idx].local_warp_valid = warped_motion_parameters(
                        pcs_ptr,
                        context_ptr->blk_ptr,
                        &mv_unit,
                        context_ptr->blk_geom,
                        context_ptr->blk_origin_x,
                        context_ptr->blk_origin_y,
                        cand_array[can_idx].ref_frame_type,
                        &cand_array[can_idx].wm_params_l0,
                        &cand_array[can_idx].num_proj_ref);

                    if (cand_array[can_idx].local_warp_valid)
                        INCRMENT_CAND_TOTAL_COUNT(can_idx);
                }
            }
        }
    }
    // NEWMV L0
#if INCREASE_WM_CANDS
#define NUM_WM_NEIGHBOUR_POS 13
    const MV neighbors[NUM_WM_NEIGHBOUR_POS] = {
        {0, 0}, {0, -1}, {1, 0}, {0, 1}, {-1, 0}, {0, -2}, {2, 0}, {0, 2}, {-2, 0}, {1, 1}, {-1, 1}, {1, -1}, {-1, 1} };
#else
    const MV neighbors[9] = {
        {0, 0}, {0, -1}, {1, 0}, {0, 1}, {-1, 0}, {0, -2}, {2, 0}, {0, 2}, {-2, 0} };
#endif
    IntMv  best_pred_mv[2] = { {0}, {0} };

    uint8_t total_me_cnt = me_results->total_me_candidate_index[context_ptr->me_block_offset];
#if ME_MEM_OPT
    const MeCandidate *me_block_results = &me_results->me_candidate_array[context_ptr->me_cand_offset];
#else
    const MeCandidate *me_block_results = me_results->me_candidate[context_ptr->me_block_offset];
#endif

    for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; ++me_candidate_index)
    {
        const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
        const uint8_t inter_direction = me_block_results_ptr->direction;
        const uint8_t list0_ref_index = me_block_results_ptr->ref_idx_l0;
        const uint8_t list1_ref_index = me_block_results_ptr->ref_idx_l1;

        /**************
            NEWMV L0
        ************* */
        if (inter_direction == 0) {
#if WARP_REF_MASKING
#if PRUNING_PER_INTER_TYPE
            if (!is_valid_unipred_ref(context_ptr, MIN(TOT_INTER_GROUP-1,WARP_GROUP), REF_LIST_0, list0_ref_index)) continue;
#else
            if (!context_ptr->ref_filtering_res[REF_LIST_0][list0_ref_index].do_ref) continue;
#endif
#endif
            if (list0_ref_index > context_ptr->md_max_ref_count - 1)
                continue;
            to_inject_mv_x =
                context_ptr
                ->sb_me_mv[context_ptr->blk_geom->blkidx_mds][REF_LIST_0][list0_ref_index][0];
            to_inject_mv_y =
                context_ptr
                ->sb_me_mv[context_ptr->blk_geom->blkidx_mds][REF_LIST_0][list0_ref_index][1];
#if !REMOVE_REF_FOR_RECT_PART
            uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
            uint8_t skip_cand = check_ref_beackout(
                context_ptr,
                to_inject_ref_type,
                context_ptr->blk_geom->shape);

            if (!skip_cand) {
#endif
#if INCREASE_WM_CANDS
                for (int i = 0; i < NUM_WM_NEIGHBOUR_POS; i++) {
#else
                for (int i = 0; i < 9; i++) {
#endif
                    cand_array[can_idx].type = INTER_MODE;
                    cand_array[can_idx].distortion_ready = 0;
                    cand_array[can_idx].use_intrabc = 0;
                    cand_array[can_idx].merge_flag = EB_FALSE;
                    cand_array[can_idx].prediction_direction[0] = (EbPredDirection)0;
                    cand_array[can_idx].inter_mode = NEWMV;
                    cand_array[can_idx].pred_mode = NEWMV;
                    cand_array[can_idx].motion_mode = WARPED_CAUSAL;
                    cand_array[can_idx].wm_params_l0.wmtype = AFFINE;
                    cand_array[can_idx].is_compound = 0;
                    cand_array[can_idx].is_interintra_used = 0;
                    cand_array[can_idx].is_new_mv = 1;
                    cand_array[can_idx].drl_index = 0;

                    if (pcs_ptr->parent_pcs_ptr->frm_hdr.allow_high_precision_mv) {
                        cand_array[can_idx].motion_vector_xl0 = to_inject_mv_x + neighbors[i].col;
                        cand_array[can_idx].motion_vector_yl0 = to_inject_mv_y + neighbors[i].row;
                    }
                    else {
                        cand_array[can_idx].motion_vector_xl0 = to_inject_mv_x + (neighbors[i].col << 1);
                        cand_array[can_idx].motion_vector_yl0 = to_inject_mv_y + (neighbors[i].row << 1);
                    }
                    cand_array[can_idx].ref_mv_index = 0;
                    cand_array[can_idx].ref_frame_type = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
                    cand_array[can_idx].ref_frame_index_l0 = list0_ref_index;
                    cand_array[can_idx].ref_frame_index_l1 = -1;
                    cand_array[can_idx].transform_type[0] = DCT_DCT;
                    cand_array[can_idx].transform_type_uv = DCT_DCT;

                    choose_best_av1_mv_pred(
                        context_ptr,
                        cand_array[can_idx].md_rate_estimation_ptr,
                        context_ptr->blk_ptr,
                        cand_array[can_idx].ref_frame_type,
                        cand_array[can_idx].is_compound,
                        cand_array[can_idx].pred_mode,
                        cand_array[can_idx].motion_vector_xl0,
                        cand_array[can_idx].motion_vector_yl0,
                        0, 0,
                        &cand_array[can_idx].drl_index,
                        best_pred_mv);

                    cand_array[can_idx].motion_vector_pred_x[REF_LIST_0] = best_pred_mv[0].as_mv.col;
                    cand_array[can_idx].motion_vector_pred_y[REF_LIST_0] = best_pred_mv[0].as_mv.row;
                    mv_0.x = cand_array[can_idx].motion_vector_xl0;
                    mv_0.y = cand_array[can_idx].motion_vector_yl0;
                    mv_unit.mv[0] = mv_0;
                    mv_unit.pred_direction = cand_array[can_idx].prediction_direction[0];
                    if (umv0_tile)
                        inside_tile = is_inside_tile_boundary(&(xd->tile), mv_0.x, mv_0.y, mi_col, mi_row, context_ptr->blk_geom->bsize);
                    if (inside_tile)
                    {
                        cand_array[can_idx].local_warp_valid = warped_motion_parameters(
                            pcs_ptr,
                            context_ptr->blk_ptr,
                            &mv_unit,
                            context_ptr->blk_geom,
                            context_ptr->blk_origin_x,
                            context_ptr->blk_origin_y,
                            cand_array[can_idx].ref_frame_type,
                            &cand_array[can_idx].wm_params_l0,
                            &cand_array[can_idx].num_proj_ref);

                        if (cand_array[can_idx].local_warp_valid)
                            INCRMENT_CAND_TOTAL_COUNT(can_idx);
                    }
                }
#if !REMOVE_REF_FOR_RECT_PART
            }
#endif
        }
        /**************
           NEWMV L1
       ************* */
        if (inter_direction == 1) {
#if WARP_REF_MASKING
#if PRUNING_PER_INTER_TYPE
            if (!is_valid_unipred_ref(context_ptr, MIN(TOT_INTER_GROUP-1,WARP_GROUP), REF_LIST_1, list1_ref_index)) continue;
#else
            if (!context_ptr->ref_filtering_res[REF_LIST_1][list1_ref_index].do_ref) continue;
#endif
#endif
            if (list1_ref_index > context_ptr->md_max_ref_count - 1)
                continue;
            to_inject_mv_x =
                context_ptr
                ->sb_me_mv[context_ptr->blk_geom->blkidx_mds][REF_LIST_1][list1_ref_index][0];
            to_inject_mv_y =
                context_ptr
                ->sb_me_mv[context_ptr->blk_geom->blkidx_mds][REF_LIST_1][list1_ref_index][1];
#if !REMOVE_REF_FOR_RECT_PART
            uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
            uint8_t skip_cand = check_ref_beackout(
                context_ptr,
                to_inject_ref_type,
                context_ptr->blk_geom->shape);
            if (!skip_cand) {
#endif
#if INCREASE_WM_CANDS
                for (int i = 0; i < NUM_WM_NEIGHBOUR_POS; i++) {
#else
                for (int i = 0; i < 9; i++) {
#endif

                    cand_array[can_idx].type = INTER_MODE;
                    cand_array[can_idx].distortion_ready = 0;
                    cand_array[can_idx].use_intrabc = 0;
                    cand_array[can_idx].merge_flag = EB_FALSE;
                    cand_array[can_idx].prediction_direction[0] = (EbPredDirection)1;
                    cand_array[can_idx].inter_mode = NEWMV;
                    cand_array[can_idx].pred_mode = NEWMV;
                    cand_array[can_idx].motion_mode = WARPED_CAUSAL;
                    cand_array[can_idx].wm_params_l0.wmtype = AFFINE;

                    cand_array[can_idx].is_compound = 0;
                    cand_array[can_idx].is_interintra_used = 0;
                    cand_array[can_idx].is_new_mv = 1;
                    cand_array[can_idx].drl_index = 0;

                    if (pcs_ptr->parent_pcs_ptr->frm_hdr.allow_high_precision_mv) {
                        cand_array[can_idx].motion_vector_xl1 = to_inject_mv_x + neighbors[i].col;
                        cand_array[can_idx].motion_vector_yl1 = to_inject_mv_y + neighbors[i].row;
                    }
                    else {
                        cand_array[can_idx].motion_vector_xl1 = to_inject_mv_x + (neighbors[i].col << 1);
                        cand_array[can_idx].motion_vector_yl1 = to_inject_mv_y + (neighbors[i].row << 1);
                    }
                    cand_array[can_idx].ref_mv_index = 0;
                    cand_array[can_idx].ref_frame_type = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
                    cand_array[can_idx].ref_frame_index_l0 = -1;
                    cand_array[can_idx].ref_frame_index_l1 = list1_ref_index;

                    cand_array[can_idx].transform_type[0] = DCT_DCT;
                    cand_array[can_idx].transform_type_uv = DCT_DCT;

                    choose_best_av1_mv_pred(
                        context_ptr,
                        cand_array[can_idx].md_rate_estimation_ptr,
                        context_ptr->blk_ptr,
                        cand_array[can_idx].ref_frame_type,
                        cand_array[can_idx].is_compound,
                        cand_array[can_idx].pred_mode,
                        cand_array[can_idx].motion_vector_xl1,
                        cand_array[can_idx].motion_vector_yl1,
                        0,
                        0,
                        &cand_array[can_idx].drl_index,
                        best_pred_mv);

                    cand_array[can_idx].motion_vector_pred_x[REF_LIST_1] = best_pred_mv[0].as_mv.col;
                    cand_array[can_idx].motion_vector_pred_y[REF_LIST_1] = best_pred_mv[0].as_mv.row;

                    mv_0.x = cand_array[can_idx].motion_vector_xl1;
                    mv_0.y = cand_array[can_idx].motion_vector_yl1;
                    mv_unit.mv[1] = mv_0;
                    mv_unit.pred_direction = cand_array[can_idx].prediction_direction[0];
                    if (umv0_tile)
                        inside_tile = is_inside_tile_boundary(&(xd->tile), mv_0.x, mv_0.y, mi_col, mi_row, context_ptr->blk_geom->bsize);
                    if (inside_tile)
                    {
                        cand_array[can_idx].local_warp_valid = warped_motion_parameters(
                            pcs_ptr,
                            context_ptr->blk_ptr,
                            &mv_unit,
                            context_ptr->blk_geom,
                            context_ptr->blk_origin_x,
                            context_ptr->blk_origin_y,
                            cand_array[can_idx].ref_frame_type,
                            &cand_array[can_idx].wm_params_l0,
                            &cand_array[can_idx].num_proj_ref);

                        if (cand_array[can_idx].local_warp_valid)
                            INCRMENT_CAND_TOTAL_COUNT(can_idx);
                    }
                }
#if !REMOVE_REF_FOR_RECT_PART
            }
#endif
        }
    }

    *cand_tot_cnt = can_idx;
}
static INLINE void setup_pred_plane(struct Buf2D *dst, BlockSize bsize, uint8_t *src, int width,
                                    int height, int stride, int mi_row, int mi_col,
                                    int subsampling_x, int subsampling_y) {
    // Offset the buffer pointer
    if (subsampling_y && (mi_row & 0x01) && (mi_size_high[bsize] == 1)) mi_row -= 1;
    if (subsampling_x && (mi_col & 0x01) && (mi_size_wide[bsize] == 1)) mi_col -= 1;

    const int x = (MI_SIZE * mi_col) >> subsampling_x;
    const int y = (MI_SIZE * mi_row) >> subsampling_y;
    dst->buf    = src + (y * stride + x); // scaled_buffer_offset(x, y, stride, scale);
    dst->buf0   = src;
    dst->width  = width;
    dst->height = height;
    dst->stride = stride;
}
void eb_av1_setup_pred_block(BlockSize sb_type, struct Buf2D dst[MAX_MB_PLANE],
                             const Yv12BufferConfig *src, int mi_row, int mi_col) {

    dst[0].buf    = src->y_buffer;
    dst[0].stride = src->y_stride;
    dst[1].buf    = src->u_buffer;
    dst[2].buf    = src->v_buffer;
    dst[1].stride = dst[2].stride = src->uv_stride;

    setup_pred_plane(dst,
                     sb_type,
                     dst[0].buf,
                     src->y_crop_width,
                     src->y_crop_height,
                     dst[0].stride,
                     mi_row,
                     mi_col,
                     0,
                     0);
}

// Values are now correlated to quantizer.
static int sad_per_bit16lut_8[QINDEX_RANGE];
#if FP_MV_COST
static int sad_per_bit_lut_10[QINDEX_RANGE];
#else
static int sad_per_bit4lut_8[QINDEX_RANGE];
#endif
extern AomVarianceFnPtr mefn_ptr[BlockSizeS_ALL];

int eb_av1_find_best_obmc_sub_pixel_tree_up(ModeDecisionContext *context_ptr, IntraBcContext *x,
                                            const AV1_COMMON *const cm, int mi_row, int mi_col,
                                            MV *bestmv, const MV *ref_mv, int allow_hp,
                                            int error_per_bit, const AomVarianceFnPtr *vfp,
                                            int forced_stop, int iters_per_step, int *mvjcost,
                                            int *mvcost[2], int *distortion, unsigned int *sse1,
                                            int is_second, int use_accurate_subpel_search);

int eb_av1_obmc_full_pixel_search(ModeDecisionContext *context_ptr, IntraBcContext *x, MV *mvp_full,
                                  int sadpb, const AomVarianceFnPtr *fn_ptr, const MV *ref_mv,
                                  MV *dst_mv, int is_second);

static void single_motion_search(PictureControlSet *pcs, ModeDecisionContext *context_ptr,
                                 ModeDecisionCandidate *candidate_ptr, const MvReferenceFrame *rf,
                                 IntMv best_pred_mv, IntraBcContext *x, BlockSize bsize, MV *ref_mv,
                                 int ref_idx, int *rate_mv) {
    (void)ref_idx;
    const Av1Common *const cm      = pcs->parent_pcs_ptr->av1_cm;
    FrameHeader *          frm_hdr = &pcs->parent_pcs_ptr->frm_hdr;
// single_motion_search supports 8bit path only
    uint32_t full_lambda = context_ptr->full_lambda_md[EB_8_BIT_MD];

    x->xd            = context_ptr->blk_ptr->av1xd;
    const int mi_row = -x->xd->mb_to_top_edge / (8 * MI_SIZE);
    const int mi_col = -x->xd->mb_to_left_edge / (8 * MI_SIZE);

    x->nmv_vec_cost  = context_ptr->md_rate_estimation_ptr->nmv_vec_cost;
    x->mv_cost_stack = context_ptr->md_rate_estimation_ptr->nmvcoststack;
    // Set up limit values for MV components.
    // Mv beyond the range do not produce new/different prediction block.
    const int mi_width   = mi_size_wide[bsize];
    const int mi_height  = mi_size_high[bsize];
    x->mv_limits.row_min = -(((mi_row + mi_height) * MI_SIZE) + AOM_INTERP_EXTEND);
    x->mv_limits.col_min = -(((mi_col + mi_width) * MI_SIZE) + AOM_INTERP_EXTEND);
    x->mv_limits.row_max = (cm->mi_rows - mi_row) * MI_SIZE + AOM_INTERP_EXTEND;
    x->mv_limits.col_max = (cm->mi_cols - mi_col) * MI_SIZE + AOM_INTERP_EXTEND;
    //set search paramters
    x->sadperbit16 = sad_per_bit16lut_8[frm_hdr->quantization_params.base_q_idx];
    x->errorperbit = full_lambda >> RD_EPB_SHIFT;
    x->errorperbit += (x->errorperbit == 0);

    int bestsme = INT_MAX;
    int sadpb   = x->sadperbit16;
    MV  mvp_full;

    MvLimits tmp_mv_limits = x->mv_limits;

    // Note: MV limits are modified here. Always restore the original values
    // after full-pixel motion search.
    eb_av1_set_mv_search_range(&x->mv_limits, ref_mv);

    mvp_full = best_pred_mv.as_mv; // mbmi->mv[0].as_mv;

    mvp_full.col >>= 3;
    mvp_full.row >>= 3;

    x->best_mv.as_int = x->second_best_mv.as_int = INVALID_MV; //D

    switch (candidate_ptr->motion_mode) {
    case OBMC_CAUSAL:
        bestsme = eb_av1_obmc_full_pixel_search(
            context_ptr, x, &mvp_full, sadpb, &mefn_ptr[bsize], ref_mv, &(x->best_mv.as_mv), 0);
        break;
    default: assert(0 && "Invalid motion mode!\n");
    }

    x->mv_limits = tmp_mv_limits;

    const int use_fractional_mv = bestsme < INT_MAX && frm_hdr->force_integer_mv == 0;
    if (use_fractional_mv) {
        int dis; /* TODO: use dis in distortion calculation later. */
        switch (candidate_ptr->motion_mode) {
        case OBMC_CAUSAL:
            eb_av1_find_best_obmc_sub_pixel_tree_up(context_ptr,
                                                    x,
                                                    cm,
                                                    mi_row,
                                                    mi_col,
                                                    &x->best_mv.as_mv,
                                                    ref_mv,
                                                    frm_hdr->allow_high_precision_mv,
                                                    x->errorperbit,
                                                    &mefn_ptr[bsize],
                                                    0, // mv.subpel_force_stop
                                                    2, //  mv.subpel_iters_per_step
                                                    x->nmv_vec_cost,
                                                    x->mv_cost_stack,
                                                    &dis,
                                                    &context_ptr->pred_sse[rf[0]],
                                                    0,
                                                    USE_8_TAPS);
            break;
        default: assert(0 && "Invalid motion mode!\n");
        }
    }
    *rate_mv = eb_av1_mv_bit_cost(
        &x->best_mv.as_mv, ref_mv, x->nmv_vec_cost, x->mv_cost_stack, MV_COST_WEIGHT);
}

void obmc_motion_refinement(PictureControlSet *pcs_ptr, struct ModeDecisionContext *context_ptr,
                            ModeDecisionCandidate *candidate, uint8_t ref_list_idx) {
    IntMv           best_pred_mv[2] = {{0}, {0}};
    IntraBcContext  x_st;
    IntraBcContext *x = &x_st;

    MacroBlockD *xd;
    xd = x->xd       = context_ptr->blk_ptr->av1xd;
    const int mi_row = -xd->mb_to_top_edge / (8 * MI_SIZE);
    const int mi_col = -xd->mb_to_left_edge / (8 * MI_SIZE);

    {
        uint8_t              ref_idx  = get_ref_frame_idx(candidate->ref_frame_type);
        uint8_t              list_idx = get_list_idx(candidate->ref_frame_type);

        assert(list_idx < MAX_NUM_OF_REF_PIC_LIST);
        EbPictureBufferDesc *reference_picture =
            ((EbReferenceObject *)pcs_ptr->ref_pic_ptr_array[list_idx][ref_idx]->object_ptr)
                ->reference_picture;

        use_scaled_rec_refs_if_needed(pcs_ptr,
                                      pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr,
                                      (EbReferenceObject *)pcs_ptr->ref_pic_ptr_array[list_idx][ref_idx]->object_ptr,
                                      &reference_picture);

        Yv12BufferConfig ref_buf;
        link_eb_to_aom_buffer_desc_8bit(reference_picture, &ref_buf);

        struct Buf2D yv12_mb[MAX_MB_PLANE];
        eb_av1_setup_pred_block(context_ptr->blk_geom->bsize, yv12_mb, &ref_buf, mi_row, mi_col);
        for (int i = 0; i < 1; ++i) x->xdplane[i].pre[0] = yv12_mb[i]; //ref in ME

        x->plane[0].src.buf  = 0; // x->xdplane[0].pre[0];
        x->plane[0].src.buf0 = 0;
    }

    IntMv best_mv;
    best_mv.as_int = 0;
    if (ref_list_idx == 0) {
        best_mv.as_mv.col = candidate->motion_vector_xl0; // to_inject_mv_x;
        best_mv.as_mv.row = candidate->motion_vector_yl0; //to_inject_mv_y;
    } else {
        best_mv.as_mv.col = candidate->motion_vector_xl1; // to_inject_mv_x;
        best_mv.as_mv.row = candidate->motion_vector_yl1; //to_inject_mv_y;
    }
    int tmp_rate_mv;

    MV ref_mv;
    ref_mv.col = candidate->motion_vector_pred_x[ref_list_idx];
    ref_mv.row = candidate->motion_vector_pred_y[ref_list_idx];

    single_motion_search(pcs_ptr,
                         context_ptr,
                         candidate,
                         (const MvReferenceFrame[]){candidate->ref_frame_type, -1},
                         best_mv,
                         x,
                         context_ptr->blk_geom->bsize,
                         &ref_mv,
                         0,
                         &tmp_rate_mv);

    if (ref_list_idx == 0) {
        candidate->motion_vector_xl0 = x->best_mv.as_mv.col;
        candidate->motion_vector_yl0 = x->best_mv.as_mv.row;
    } else {
        candidate->motion_vector_xl1 = x->best_mv.as_mv.col;
        candidate->motion_vector_yl1 = x->best_mv.as_mv.row;
    }

    choose_best_av1_mv_pred(
        context_ptr,
        candidate->md_rate_estimation_ptr,
        context_ptr->blk_ptr,
        candidate->ref_frame_type,
        candidate->is_compound,
        candidate->pred_mode,
        ref_list_idx == 0 ? candidate->motion_vector_xl0 : candidate->motion_vector_xl1,
        ref_list_idx == 0 ? candidate->motion_vector_yl0 : candidate->motion_vector_yl1,
        0,
        0,
        &candidate->drl_index,
        best_pred_mv);

    if (ref_list_idx == 0) {
        candidate->motion_vector_pred_x[REF_LIST_0] = best_pred_mv[0].as_mv.col;
        candidate->motion_vector_pred_y[REF_LIST_0] = best_pred_mv[0].as_mv.row;
    } else {
        candidate->motion_vector_pred_x[REF_LIST_1] = best_pred_mv[0].as_mv.col;
        candidate->motion_vector_pred_y[REF_LIST_1] = best_pred_mv[0].as_mv.row;
    }
}

void inject_new_candidates(const SequenceControlSet *  scs_ptr,
                           struct ModeDecisionContext *context_ptr, PictureControlSet *pcs_ptr,
                           EbBool is_compound_enabled, EbBool allow_bipred, uint32_t me_sb_addr,
                           uint32_t me_block_offset, uint32_t *candidate_total_cnt) {
    ModeDecisionCandidate *cand_array      = context_ptr->fast_candidate_array;
    IntMv                  best_pred_mv[2] = {{0}, {0}};
    uint32_t               cand_total_cnt  = (*candidate_total_cnt);
#if DECOUPLE_ME_RES
    const MeSbResults *me_results =
        pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[me_sb_addr];
#else
    const MeSbResults *me_results       = pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr];
#endif
    uint8_t            total_me_cnt     = me_results->total_me_candidate_index[me_block_offset];
#if ME_MEM_OPT
    const MeCandidate *me_block_results = &me_results->me_candidate_array[context_ptr->me_cand_offset];
#else
    const MeCandidate *me_block_results = me_results->me_candidate[me_block_offset];
#endif
    MacroBlockD *      xd               = context_ptr->blk_ptr->av1xd;
    int                inside_tile      = 1;
    int                umv0tile         = (scs_ptr->static_config.unrestricted_motion_vector == 0);
    uint32_t           mi_row           = context_ptr->blk_origin_y >> MI_SIZE_LOG2;
    uint32_t           mi_col           = context_ptr->blk_origin_x >> MI_SIZE_LOG2;
    BlockSize          bsize            = context_ptr->blk_geom->bsize; // bloc size
    MD_COMP_TYPE       cur_type; //NN
#if INTER_COMP_REDESIGN
    MD_COMP_TYPE       tot_comp_types = context_ptr->compound_types_to_try;
#else
    MD_COMP_TYPE       tot_comp_types = (pcs_ptr->parent_pcs_ptr->compound_mode == 1 ||
                                   context_ptr->compound_types_to_try == MD_COMP_AVG)
                                      ? MD_COMP_AVG
                                      : (bsize >= BLOCK_8X8 && bsize <= BLOCK_32X32)
                                            ? context_ptr->compound_types_to_try
                                            : context_ptr->compound_types_to_try == MD_COMP_WEDGE
                                                  ? MD_COMP_DIFF0
                                                  : context_ptr->compound_types_to_try;
#endif
#if !INTER_COMP_REDESIGN
    if (context_ptr->source_variance < context_ptr->inter_inter_wedge_variance_th)
        tot_comp_types = MIN(tot_comp_types, MD_COMP_DIFF0);

#else
#if !OPT_4
    if (context_ptr->source_variance < context_ptr->inter_comp_ctrls.wedge_variance_th)
        tot_comp_types = MIN(tot_comp_types, MD_COMP_DIFF0);
#endif
#endif
    for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; ++me_candidate_index) {
        const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
        const uint8_t      inter_direction      = me_block_results_ptr->direction;
        const uint8_t      list0_ref_index      = me_block_results_ptr->ref_idx_l0;
        const uint8_t      list1_ref_index      = me_block_results_ptr->ref_idx_l1;

        /**************
            NEWMV L0
        ************* */
        if (inter_direction == 0) {
#if NEW_MV_REF_MASKING
#if PRUNING_PER_INTER_TYPE
            if (!is_valid_unipred_ref(context_ptr, MIN(TOT_INTER_GROUP-1,PA_ME_GROUP), REF_LIST_0, list0_ref_index))
#else
            if (!context_ptr->ref_filtering_res[REF_LIST_0][list0_ref_index].do_ref)
#endif
                continue;
#endif
            if (list0_ref_index > context_ptr->md_max_ref_count - 1) continue;
            int16_t to_inject_mv_x =
                context_ptr
                ->sb_me_mv[context_ptr->blk_geom->blkidx_mds][REF_LIST_0][list0_ref_index][0];
            int16_t to_inject_mv_y =
                context_ptr
                ->sb_me_mv[context_ptr->blk_geom->blkidx_mds][REF_LIST_0][list0_ref_index][1];
            uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
#if !REMOVE_REF_FOR_RECT_PART
            uint8_t skip_cand =
                check_ref_beackout(context_ptr, to_inject_ref_type, context_ptr->blk_geom->shape);
#endif
            inside_tile = 1;
            if (umv0tile)
                inside_tile = is_inside_tile_boundary(&(xd->tile),
                                                      to_inject_mv_x,
                                                      to_inject_mv_y,
                                                      mi_col,
                                                      mi_row,
                                                      context_ptr->blk_geom->bsize);
#if REMOVE_REF_FOR_RECT_PART
            uint8_t skip_cand = (!inside_tile);
#else
            skip_cand = skip_cand || (!inside_tile);
#endif

            if (!skip_cand &&
                (context_ptr->injected_mv_count_l0 == 0 ||
                 mrp_is_already_injected_mv_l0(
                     context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) ==
                     EB_FALSE)) {
                uint8_t inter_type;
                uint8_t is_ii_allowed = svt_is_interintra_allowed(
                    context_ptr->md_inter_intra_level,
                    bsize,
                    NEWMV,
                    (const MvReferenceFrame[]){to_inject_ref_type, -1});
#if INTRA_COMPOUND_OPT
#if DECOUPLE_ME_RES
                if (context_ptr->md_inter_intra_level > 2)
                    if (pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[me_sb_addr]->do_comp[0][list0_ref_index] == 0)
                        is_ii_allowed = 0;
#else
                if (context_ptr->md_inter_intra_level > 2)
                    if  (pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr]->do_comp[0][list0_ref_index] == 0 )
                        is_ii_allowed = 0;
#endif
#endif
                uint8_t tot_inter_types = is_ii_allowed ? II_COUNT : 1;
                uint8_t is_obmc_allowed =
                    obmc_motion_mode_allowed(
                        pcs_ptr, context_ptr, bsize, to_inject_ref_type, -1, NEWMV) == OBMC_CAUSAL;

#if OBMC_FAST
                is_obmc_allowed = context_ptr->obmc_ctrls.me_count == 0 ? 0 :
                    me_candidate_index > context_ptr->obmc_ctrls.me_count - 1 ? 0 :
                    is_obmc_allowed;
                tot_inter_types = is_obmc_allowed ? tot_inter_types + 1 : tot_inter_types;
#else
                tot_inter_types = is_obmc_allowed && context_ptr->md_pic_obmc_mode <= 2
                                      ? tot_inter_types + 1
                                      : tot_inter_types;

#endif
                for (inter_type = 0; inter_type < tot_inter_types; inter_type++) {
                    cand_array[cand_total_cnt].type                    = INTER_MODE;
                    cand_array[cand_total_cnt].distortion_ready        = 0;
                    cand_array[cand_total_cnt].use_intrabc             = 0;
                    cand_array[cand_total_cnt].merge_flag              = EB_FALSE;
                    cand_array[cand_total_cnt].prediction_direction[0] = (EbPredDirection)0;
                    cand_array[cand_total_cnt].inter_mode              = NEWMV;
                    cand_array[cand_total_cnt].pred_mode               = NEWMV;
                    cand_array[cand_total_cnt].motion_mode             = SIMPLE_TRANSLATION;
                    cand_array[cand_total_cnt].is_compound             = 0;
                    cand_array[cand_total_cnt].is_new_mv               = 1;
                    cand_array[cand_total_cnt].drl_index               = 0;

                    // Set the MV to ME result
                    cand_array[cand_total_cnt].motion_vector_xl0 = to_inject_mv_x;
                    cand_array[cand_total_cnt].motion_vector_yl0 = to_inject_mv_y;

                    // will be needed later by the rate estimation
                    cand_array[cand_total_cnt].ref_mv_index   = 0;
                    cand_array[cand_total_cnt].ref_frame_type =
                        svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
                    cand_array[cand_total_cnt].ref_frame_index_l0 = list0_ref_index;
                    cand_array[cand_total_cnt].ref_frame_index_l1 = -1;

                    cand_array[cand_total_cnt].transform_type[0] = DCT_DCT;
                    cand_array[cand_total_cnt].transform_type_uv = DCT_DCT;

                    choose_best_av1_mv_pred(context_ptr,
                                            cand_array[cand_total_cnt].md_rate_estimation_ptr,
                                            context_ptr->blk_ptr,
                                            cand_array[cand_total_cnt].ref_frame_type,
                                            cand_array[cand_total_cnt].is_compound,
                                            cand_array[cand_total_cnt].pred_mode,
                                            cand_array[cand_total_cnt].motion_vector_xl0,
                                            cand_array[cand_total_cnt].motion_vector_yl0,
                                            0,
                                            0,
                                            &cand_array[cand_total_cnt].drl_index,
                                            best_pred_mv);

                    cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_0] =
                        best_pred_mv[0].as_mv.col;
                    cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_0] =
                        best_pred_mv[0].as_mv.row;

                    if (inter_type == 0) {
                        cand_array[cand_total_cnt].is_interintra_used = 0;
                        cand_array[cand_total_cnt].motion_mode        = SIMPLE_TRANSLATION;
                    } else {
                        if (is_ii_allowed) {
                            if (inter_type == 1) {
                                inter_intra_search(
                                    pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);
                                cand_array[cand_total_cnt].is_interintra_used   = 1;
                                cand_array[cand_total_cnt].use_wedge_interintra = 1;
                            } else if (inter_type == 2) {
                                cand_array[cand_total_cnt].is_interintra_used = 1;
                                cand_array[cand_total_cnt].interintra_mode =
                                    cand_array[cand_total_cnt - 1].interintra_mode;
                                cand_array[cand_total_cnt].use_wedge_interintra = 0;
                            }
                        }

                        if (is_obmc_allowed && inter_type == tot_inter_types - 1) {
                            cand_array[cand_total_cnt].is_interintra_used = 0;
                            cand_array[cand_total_cnt].motion_mode        = OBMC_CAUSAL;

                            obmc_motion_refinement(
                                pcs_ptr, context_ptr, &cand_array[cand_total_cnt], REF_LIST_0);
                        }
                    }

                    INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                }
                context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] =
                    to_inject_mv_x;
                context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] =
                    to_inject_mv_y;
                context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] =
                    to_inject_ref_type;
                ++context_ptr->injected_mv_count_l0;
#if !PD0_INTER_CAND
                if (context_ptr->best_me_cand_only_flag) break;
#endif
            }
        }

        if (is_compound_enabled) {
            /**************
               NEWMV L1
           ************* */
            if (inter_direction == 1) {
#if NEW_MV_REF_MASKING
#if PRUNING_PER_INTER_TYPE
                if (!is_valid_unipred_ref(context_ptr, MIN(TOT_INTER_GROUP-1,PA_ME_GROUP), REF_LIST_1, list1_ref_index))
#else
                if (!context_ptr->ref_filtering_res[REF_LIST_1][list1_ref_index].do_ref)
#endif
                    continue;
#endif
                if (list1_ref_index > context_ptr->md_max_ref_count - 1) continue;
                int16_t to_inject_mv_x = context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                    [REF_LIST_1][list1_ref_index][0];
                int16_t to_inject_mv_y = context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                    [REF_LIST_1][list1_ref_index][1];
                uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
#if !REMOVE_REF_FOR_RECT_PART
                uint8_t skip_cand          = check_ref_beackout(
                    context_ptr, to_inject_ref_type, context_ptr->blk_geom->shape);
#endif

                inside_tile = 1;
                if (umv0tile)
                    inside_tile = is_inside_tile_boundary(&(xd->tile),
                                                          to_inject_mv_x,
                                                          to_inject_mv_y,
                                                          mi_col,
                                                          mi_row,
                                                          context_ptr->blk_geom->bsize);
#if REMOVE_REF_FOR_RECT_PART
                uint8_t skip_cand = !inside_tile;
#else
                skip_cand = skip_cand || !inside_tile;
#endif

                if (!skip_cand &&
                    (context_ptr->injected_mv_count_l1 == 0 ||
                     mrp_is_already_injected_mv_l1(
                         context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) ==
                         EB_FALSE)) {

                    uint8_t inter_type;
                    uint8_t is_ii_allowed = svt_is_interintra_allowed(
                        context_ptr->md_inter_intra_level,
                        bsize,
                        NEWMV,
                        (const MvReferenceFrame[]){to_inject_ref_type, -1});
#if INTRA_COMPOUND_OPT
#if DECOUPLE_ME_RES
                    if (context_ptr->md_inter_intra_level > 2)
                        if (pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[me_sb_addr]->do_comp[1][list1_ref_index] == 0)
                            is_ii_allowed = 0;
#else
                    if (context_ptr->md_inter_intra_level > 2)
                        if  ( pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr]->do_comp[1][list1_ref_index] == 0)
                                is_ii_allowed = 0;
#endif
#endif
                    uint8_t tot_inter_types = is_ii_allowed ? II_COUNT : 1;
                    uint8_t is_obmc_allowed =
                        obmc_motion_mode_allowed(
                            pcs_ptr, context_ptr, bsize, to_inject_ref_type, -1, NEWMV) == OBMC_CAUSAL;
#if OBMC_FAST
                    is_obmc_allowed =context_ptr->obmc_ctrls.me_count == 0 ? 0 :
                        me_candidate_index > context_ptr->obmc_ctrls.me_count - 1 ? 0 :
                        is_obmc_allowed;
                    tot_inter_types = is_obmc_allowed ? tot_inter_types + 1 : tot_inter_types;
#else
                    tot_inter_types = is_obmc_allowed && pcs_ptr->parent_pcs_ptr->pic_obmc_mode <= 2
                                          ? tot_inter_types + 1
                                          : tot_inter_types;
#endif
                    for (inter_type = 0; inter_type < tot_inter_types; inter_type++) {
                        cand_array[cand_total_cnt].type                    = INTER_MODE;
                        cand_array[cand_total_cnt].distortion_ready        = 0;
                        cand_array[cand_total_cnt].use_intrabc             = 0;
                        cand_array[cand_total_cnt].merge_flag              = EB_FALSE;
                        cand_array[cand_total_cnt].prediction_direction[0] = (EbPredDirection)1;
                        cand_array[cand_total_cnt].inter_mode              = NEWMV;
                        cand_array[cand_total_cnt].pred_mode               = NEWMV;
                        cand_array[cand_total_cnt].motion_mode             = SIMPLE_TRANSLATION;
                        cand_array[cand_total_cnt].is_compound             = 0;
                        cand_array[cand_total_cnt].is_new_mv               = 1;
                        cand_array[cand_total_cnt].drl_index               = 0;

                        // Set the MV to ME result
                        cand_array[cand_total_cnt].motion_vector_xl1 = to_inject_mv_x;
                        cand_array[cand_total_cnt].motion_vector_yl1 = to_inject_mv_y;

                        // will be needed later by the rate estimation
                        cand_array[cand_total_cnt].ref_mv_index   = 0;
                        cand_array[cand_total_cnt].ref_frame_type =
                            svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
                        cand_array[cand_total_cnt].ref_frame_index_l0 = -1;
                        cand_array[cand_total_cnt].ref_frame_index_l1 = list1_ref_index;

                        cand_array[cand_total_cnt].transform_type[0] = DCT_DCT;
                        cand_array[cand_total_cnt].transform_type_uv = DCT_DCT;

                        choose_best_av1_mv_pred(context_ptr,
                                                cand_array[cand_total_cnt].md_rate_estimation_ptr,
                                                context_ptr->blk_ptr,
                                                cand_array[cand_total_cnt].ref_frame_type,
                                                cand_array[cand_total_cnt].is_compound,
                                                cand_array[cand_total_cnt].pred_mode,
                                                cand_array[cand_total_cnt].motion_vector_xl1,
                                                cand_array[cand_total_cnt].motion_vector_yl1,
                                                0,
                                                0,
                                                &cand_array[cand_total_cnt].drl_index,
                                                best_pred_mv);

                        cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_1] =
                            best_pred_mv[0].as_mv.col;
                        cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_1] =
                            best_pred_mv[0].as_mv.row;
                        if (inter_type == 0) {
                            cand_array[cand_total_cnt].is_interintra_used = 0;
                            cand_array[cand_total_cnt].motion_mode        = SIMPLE_TRANSLATION;
                        } else {
                            if (is_ii_allowed) {
                                if (inter_type == 1) {
                                    inter_intra_search(
                                        pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);
                                    cand_array[cand_total_cnt].is_interintra_used   = 1;
                                    cand_array[cand_total_cnt].use_wedge_interintra = 1;
                                } else if (inter_type == 2) {
                                    cand_array[cand_total_cnt].is_interintra_used = 1;
                                    cand_array[cand_total_cnt].interintra_mode =
                                        cand_array[cand_total_cnt - 1].interintra_mode;
                                    cand_array[cand_total_cnt].use_wedge_interintra = 0;
                                }
                            }
                            if (is_obmc_allowed && inter_type == tot_inter_types - 1) {
                                cand_array[cand_total_cnt].is_interintra_used = 0;
                                cand_array[cand_total_cnt].motion_mode        = OBMC_CAUSAL;

                                obmc_motion_refinement(
                                    pcs_ptr, context_ptr, &cand_array[cand_total_cnt], REF_LIST_1);
                            }
                        }
                        INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                    }
                    context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] =
                        to_inject_mv_x;
                    context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] =
                        to_inject_mv_y;
                    context_ptr->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] =
                        to_inject_ref_type;
                    ++context_ptr->injected_mv_count_l1;
#if !PD0_INTER_CAND
                    if (context_ptr->best_me_cand_only_flag) break;
#endif
                }
            }
            /**************
               NEW_NEWMV
            ************* */
            if (allow_bipred) {
#if NEW_MV_REF_MASKING
#if PRUNING_PER_INTER_TYPE
                if (inter_direction == 2) {
                if (!is_valid_bipred_ref(context_ptr, PA_ME_GROUP, me_block_results_ptr->ref0_list, list0_ref_index, me_block_results_ptr->ref1_list, list1_ref_index))
#else
                if (!context_ptr->ref_filtering_res[me_block_results_ptr->ref0_list][list0_ref_index].do_ref || !context_ptr->ref_filtering_res[me_block_results_ptr->ref1_list][list1_ref_index].do_ref)
#endif
                    continue;
#endif
#if !PRUNING_PER_INTER_TYPE
                if (inter_direction == 2) {
#endif
                    if (list0_ref_index > context_ptr->md_max_ref_count - 1 ||
                        list1_ref_index > context_ptr->md_max_ref_count - 1)
                        continue;
                    int16_t to_inject_mv_x_l0 =
                        context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                        [me_block_results_ptr->ref0_list][list0_ref_index][0];
                    int16_t to_inject_mv_y_l0 =
                        context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                        [me_block_results_ptr->ref0_list][list0_ref_index][1];
                    int16_t to_inject_mv_x_l1 =
                        context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                        [me_block_results_ptr->ref1_list][list1_ref_index][0];
                    int16_t to_inject_mv_y_l1 =
                        context_ptr->sb_me_mv[context_ptr->blk_geom->blkidx_mds]
                        [me_block_results_ptr->ref1_list][list1_ref_index][1];
                    uint8_t to_inject_ref_type = av1_ref_frame_type((const MvReferenceFrame[]){
                        svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index),
                        svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index)});
#if !REMOVE_REF_FOR_RECT_PART
                    uint8_t skip_cand          = check_ref_beackout(
                        context_ptr, to_inject_ref_type, context_ptr->blk_geom->shape);
#endif

                    inside_tile = 1;
                    if (umv0tile) {
                        inside_tile = is_inside_tile_boundary(&(xd->tile),
                                                              to_inject_mv_x_l0,
                                                              to_inject_mv_y_l0,
                                                              mi_col,
                                                              mi_row,
                                                              context_ptr->blk_geom->bsize) &&
                                      is_inside_tile_boundary(&(xd->tile),
                                                              to_inject_mv_x_l1,
                                                              to_inject_mv_y_l1,
                                                              mi_col,
                                                              mi_row,
                                                              context_ptr->blk_geom->bsize);
                    }
#if REMOVE_REF_FOR_RECT_PART
                    uint8_t skip_cand = (!inside_tile);
#else
                    skip_cand = skip_cand || (!inside_tile);
#endif
                    if (!skip_cand &&
                        (context_ptr->injected_mv_count_bipred == 0 ||
                         mrp_is_already_injected_mv_bipred(context_ptr,
                                                           to_inject_mv_x_l0,
                                                           to_inject_mv_y_l0,
                                                           to_inject_mv_x_l1,
                                                           to_inject_mv_y_l1,
                                                           to_inject_ref_type) == EB_FALSE)) {
#if !INTER_COMP_REDESIGN
                        context_ptr->variance_ready = 0;
#endif
#if INTER_COMP_REDESIGN
#if DECOUPLE_ME_RES
                        if (context_ptr->inter_comp_ctrls.mrp_pruning_w_distortion)
                            if (pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[me_sb_addr]->do_comp[0][list0_ref_index] == 0 ||
                                pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[me_sb_addr]->do_comp[1][list1_ref_index] == 0)
                                tot_comp_types = MD_COMP_AVG;
#else
                        if (context_ptr->inter_comp_ctrls.mrp_pruning_w_distortion)
                            if  (pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr]->do_comp[0][list0_ref_index] == 0 ||
                                    pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr]->do_comp[1][list1_ref_index] == 0)
                                    tot_comp_types = MD_COMP_AVG;
#endif

#endif
                        for (cur_type = MD_COMP_AVG; cur_type <= tot_comp_types; cur_type++) {
#if !INTER_COMP_REDESIGN
                            if (cur_type == MD_COMP_WEDGE &&
                                    get_wedge_params_bits(context_ptr->blk_geom->bsize) == 0)
                                continue;
#endif
#if !REMOVE_SIMILARITY_FEATS
                            // If two predictors are very similar, skip wedge compound mode search
#if INTER_COMP_REDESIGN
                            if (cur_type == MD_COMP_WEDGE && context_ptr->inter_comp_ctrls.similar_predictions)
#else
                            if (context_ptr->variance_ready)
#endif
                                if (context_ptr->prediction_mse < 8)
                                    continue;
#endif
                            cand_array[cand_total_cnt].type = INTER_MODE;

                            cand_array[cand_total_cnt].distortion_ready = 0;
                            cand_array[cand_total_cnt].use_intrabc      = 0;

                            cand_array[cand_total_cnt].merge_flag = EB_FALSE;

                            cand_array[cand_total_cnt].is_new_mv  = 1;
                            cand_array[cand_total_cnt].drl_index = 0;

                            // Set the MV to ME result

                            cand_array[cand_total_cnt].motion_vector_xl0 = to_inject_mv_x_l0;
                            cand_array[cand_total_cnt].motion_vector_yl0 = to_inject_mv_y_l0;
                            cand_array[cand_total_cnt].motion_vector_xl1 = to_inject_mv_x_l1;
                            cand_array[cand_total_cnt].motion_vector_yl1 = to_inject_mv_y_l1;

                            // will be needed later by the rate estimation
                            cand_array[cand_total_cnt].ref_mv_index            = 0;
                            cand_array[cand_total_cnt].inter_mode              = NEW_NEWMV;
                            cand_array[cand_total_cnt].pred_mode               = NEW_NEWMV;
                            cand_array[cand_total_cnt].motion_mode             = SIMPLE_TRANSLATION;
                            cand_array[cand_total_cnt].is_compound             = 1;
                            cand_array[cand_total_cnt].is_interintra_used      = 0;
                            cand_array[cand_total_cnt].prediction_direction[0] = (EbPredDirection)2;
                            cand_array[cand_total_cnt].ref_frame_type = av1_ref_frame_type(
                                (const MvReferenceFrame[]){
                                    svt_get_ref_frame_type(me_block_results_ptr->ref0_list,
                                                           list0_ref_index),
                                    svt_get_ref_frame_type(me_block_results_ptr->ref1_list,
                                                           list1_ref_index)});
                            cand_array[cand_total_cnt].ref_frame_index_l0 = list0_ref_index;
                            cand_array[cand_total_cnt].ref_frame_index_l1 = list1_ref_index;
                            cand_array[cand_total_cnt].transform_type[0]  = DCT_DCT;
                            cand_array[cand_total_cnt].transform_type_uv  = DCT_DCT;
                            choose_best_av1_mv_pred(
                                context_ptr,
                                cand_array[cand_total_cnt].md_rate_estimation_ptr,
                                context_ptr->blk_ptr,
                                cand_array[cand_total_cnt].ref_frame_type,
                                cand_array[cand_total_cnt].is_compound,
                                cand_array[cand_total_cnt].pred_mode,
                                cand_array[cand_total_cnt].motion_vector_xl0,
                                cand_array[cand_total_cnt].motion_vector_yl0,
                                cand_array[cand_total_cnt].motion_vector_xl1,
                                cand_array[cand_total_cnt].motion_vector_yl1,
                                &cand_array[cand_total_cnt].drl_index,
                                best_pred_mv);

                            cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_0] =
                                best_pred_mv[0].as_mv.col;
                            cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_0] =
                                best_pred_mv[0].as_mv.row;
                            cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_1] =
                                best_pred_mv[1].as_mv.col;
                            cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_1] =
                                best_pred_mv[1].as_mv.row;
                            //NEW_NEW
#if INTER_COMP_REDESIGN
                            if (cur_type == MD_COMP_AVG && tot_comp_types > MD_COMP_AVG)

                                calc_pred_masked_compound(
                                    pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);
#if !REMOVE_SIMILARITY_FEATS
                            if (context_ptr->inter_comp_ctrls.similar_predictions)
                                if (cur_type > MD_COMP_AVG &&
                                    context_ptr->prediction_mse <=
                                    context_ptr->inter_comp_ctrls.similar_predictions_th )
                                    continue;
#endif
#endif
                            determine_compound_mode(
                                pcs_ptr, context_ptr, &cand_array[cand_total_cnt], cur_type);
                            INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);

                            context_ptr->injected_mv_x_bipred_l0_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                            context_ptr->injected_mv_y_bipred_l0_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                            context_ptr->injected_mv_x_bipred_l1_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                            context_ptr->injected_mv_y_bipred_l1_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                            context_ptr->injected_ref_type_bipred_array
                                [context_ptr->injected_mv_count_bipred] = to_inject_ref_type;
                            ++context_ptr->injected_mv_count_bipred;
#if !PD0_INTER_CAND
                            if (context_ptr->best_me_cand_only_flag) break;
#endif
                        }
                    }
                }
            }
        }
    }
    // update the total number of candidates injected
    (*candidate_total_cnt) = cand_total_cnt;
}
#if IMPROVE_GMV
void inject_global_candidates(const SequenceControlSet *  scs_ptr,
    struct ModeDecisionContext *context_ptr, PictureControlSet *pcs_ptr,
    EbBool is_compound_enabled, EbBool allow_bipred, uint32_t *candidate_total_cnt) {

    ModeDecisionCandidate *cand_array = context_ptr->fast_candidate_array;
    uint32_t cand_total_cnt = (*candidate_total_cnt);
    MD_COMP_TYPE cur_type;
    MD_COMP_TYPE tot_comp_types = context_ptr->compound_types_to_try;
    uint8_t inj_mv;
    int inside_tile = 1;
    MacroBlockD *xd = context_ptr->blk_ptr->av1xd;
    int umv0tile = (scs_ptr->static_config.unrestricted_motion_vector == 0);
    uint32_t  mi_row = context_ptr->blk_origin_y >> MI_SIZE_LOG2;
    uint32_t  mi_col = context_ptr->blk_origin_x >> MI_SIZE_LOG2;
    BlockSize bsize = context_ptr->blk_geom->bsize; // bloc size

    for (uint32_t ref_it = 0; ref_it < pcs_ptr->parent_pcs_ptr->tot_ref_frame_types; ++ref_it) {

        MvReferenceFrame ref_pair = pcs_ptr->parent_pcs_ptr->ref_frame_type_arr[ref_it];
        MvReferenceFrame rf[2];
        av1_set_ref_frame(rf, ref_pair);

        //single ref/list
        if (rf[1] == NONE_FRAME) {
            MvReferenceFrame frame_type = rf[0];
            uint8_t          list_idx = get_list_idx(rf[0]);
            uint8_t          ref_idx = get_ref_frame_idx(rf[0]);


            if (!is_valid_unipred_ref(context_ptr, GLOBAL_GROUP, list_idx, ref_idx)) continue;

            if (ref_idx > context_ptr->md_max_ref_count - 1)
                continue;

            // Get gm params
            EbWarpedMotionParams *gm_params = &pcs_ptr->parent_pcs_ptr->global_motion[frame_type];

            IntMv mv = gm_get_motion_vector_enc(
                gm_params,
                pcs_ptr->parent_pcs_ptr->frm_hdr.allow_high_precision_mv,
                context_ptr->blk_geom->bsize,
                mi_col,
                mi_row,
                0 /* force_integer_mv */);

            int16_t to_inject_mv_x = mv.as_mv.col;
            int16_t to_inject_mv_y = mv.as_mv.row;

            inj_mv = 1; // Always test GLOBAL even if MV already injected as rate diff might be significant
            if (umv0tile)
                inside_tile = is_inside_tile_boundary(&(xd->tile),
                    to_inject_mv_x,
                    to_inject_mv_y,
                    mi_col,
                    mi_row,
                    context_ptr->blk_geom->bsize);

            inj_mv = inj_mv && inside_tile;

            if (inj_mv && (((gm_params->wmtype > TRANSLATION && context_ptr->blk_geom->bwidth >= 8 && context_ptr->blk_geom->bheight >= 8) || gm_params->wmtype <= TRANSLATION))) {

                uint8_t inter_type;
                uint8_t is_ii_allowed = svt_is_interintra_allowed(
                    context_ptr->md_inter_intra_level, bsize, GLOBALMV, rf);

                uint8_t tot_inter_types = is_ii_allowed ? II_COUNT : 1;

                for (inter_type = 0; inter_type < tot_inter_types; inter_type++) {
                    cand_array[cand_total_cnt].type = INTER_MODE;

                    cand_array[cand_total_cnt].inter_mode = GLOBALMV;
                    cand_array[cand_total_cnt].pred_mode = GLOBALMV;
                    cand_array[cand_total_cnt].motion_mode = gm_params->wmtype > TRANSLATION
                        ? WARPED_CAUSAL
                        : SIMPLE_TRANSLATION;

                    cand_array[cand_total_cnt].wm_params_l0 = *gm_params;
                    cand_array[cand_total_cnt].wm_params_l1 = *gm_params;

                    cand_array[cand_total_cnt].is_compound = 0;
                    cand_array[cand_total_cnt].distortion_ready = 0;
                    cand_array[cand_total_cnt].use_intrabc = 0;
                    cand_array[cand_total_cnt].merge_flag = EB_FALSE;
                    cand_array[cand_total_cnt].prediction_direction[0] = list_idx;
                    cand_array[cand_total_cnt].is_new_mv = 0;
                    if (list_idx == 0) {
                        cand_array[cand_total_cnt].motion_vector_xl0 = to_inject_mv_x;
                        cand_array[cand_total_cnt].motion_vector_yl0 = to_inject_mv_y;
                        context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] =
                            to_inject_mv_x;
                        context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] =
                            to_inject_mv_y;
                        context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] =
                            frame_type;
                        ++context_ptr->injected_mv_count_l0;
                    }
                    else {
                        cand_array[cand_total_cnt].motion_vector_xl1 = to_inject_mv_x;
                        cand_array[cand_total_cnt].motion_vector_yl1 = to_inject_mv_y;
                        context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] =
                            to_inject_mv_x;
                        context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] =
                            to_inject_mv_y;
                        context_ptr->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] =
                            frame_type;
                        ++context_ptr->injected_mv_count_l1;
                    }

                    cand_array[cand_total_cnt].drl_index = 0;
                    cand_array[cand_total_cnt].ref_mv_index = 0;

                    cand_array[cand_total_cnt].ref_frame_type = frame_type;
                    cand_array[cand_total_cnt].ref_frame_index_l0 = (list_idx == 0) ? ref_idx : -1;
                    cand_array[cand_total_cnt].ref_frame_index_l1 = (list_idx == 1) ? ref_idx : -1;
                    cand_array[cand_total_cnt].transform_type[0] = DCT_DCT;
                    cand_array[cand_total_cnt].transform_type_uv = DCT_DCT;
                    if (inter_type == 0) {
                        cand_array[cand_total_cnt].is_interintra_used = 0;
                    }
                    else {
                        if (is_ii_allowed) {
                            if (inter_type == 1) {
                                inter_intra_search(
                                    pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);
                                cand_array[cand_total_cnt].is_interintra_used = 1;
                                cand_array[cand_total_cnt].use_wedge_interintra = 1;

                            }
                            else if (inter_type == 2) {
                                cand_array[cand_total_cnt].is_interintra_used = 1;
                                cand_array[cand_total_cnt].interintra_mode =
                                    cand_array[cand_total_cnt - 1].interintra_mode;
                                cand_array[cand_total_cnt].use_wedge_interintra = 0;
                            }
                        }
                    }
                    INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                }
                if (list_idx == 0) {
                    cand_array[cand_total_cnt].motion_vector_xl0 = to_inject_mv_x;
                    cand_array[cand_total_cnt].motion_vector_yl0 = to_inject_mv_y;
                    context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] =
                        to_inject_mv_x;
                    context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] =
                        to_inject_mv_y;
                    context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] =
                        frame_type;
                    ++context_ptr->injected_mv_count_l0;
                }
                else {
                    cand_array[cand_total_cnt].motion_vector_xl1 = to_inject_mv_x;
                    cand_array[cand_total_cnt].motion_vector_yl1 = to_inject_mv_y;
                    context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] =
                        to_inject_mv_x;
                    context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] =
                        to_inject_mv_y;
                    context_ptr->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] =
                        frame_type;
                    ++context_ptr->injected_mv_count_l1;
                }
            }
        }
        else if (is_compound_enabled && allow_bipred) {

            uint8_t ref_idx_0 = get_ref_frame_idx(rf[0]);
            uint8_t ref_idx_1 = get_ref_frame_idx(rf[1]);
            uint8_t list_idx_0 = get_list_idx(rf[0]);
            uint8_t list_idx_1 = get_list_idx(rf[1]);

            if (!is_valid_bipred_ref(
                context_ptr, GLOBAL_GROUP, list_idx_0, ref_idx_0, list_idx_1, ref_idx_1)) return;

            if (ref_idx_0 > context_ptr->md_max_ref_count - 1 ||
                ref_idx_1 > context_ptr->md_max_ref_count - 1)
                return;

            // Get gm params
            EbWarpedMotionParams *gm_params_0 =
                &pcs_ptr->parent_pcs_ptr->global_motion[svt_get_ref_frame_type(
                    list_idx_0, ref_idx_0)];

            EbWarpedMotionParams *gm_params_1 =
                &pcs_ptr->parent_pcs_ptr->global_motion[svt_get_ref_frame_type(
                    list_idx_1, ref_idx_1)];

            IntMv mv_0 = gm_get_motion_vector_enc(
                gm_params_0,
                pcs_ptr->parent_pcs_ptr->frm_hdr.allow_high_precision_mv,
                context_ptr->blk_geom->bsize,
                mi_col,
                mi_row,
                0 /* force_integer_mv */);

            int16_t to_inject_mv_x_l0 = mv_0.as_mv.col;
            int16_t to_inject_mv_y_l0 = mv_0.as_mv.row;

            IntMv mv_1 = gm_get_motion_vector_enc(
                gm_params_1,
                pcs_ptr->parent_pcs_ptr->frm_hdr.allow_high_precision_mv,
                context_ptr->blk_geom->bsize,
                mi_col,
                mi_row,
                0 /* force_integer_mv */);

            int16_t to_inject_mv_x_l1 = mv_1.as_mv.col;
            int16_t to_inject_mv_y_l1 = mv_1.as_mv.row;

            inj_mv = 1; // Always test GLOBAL-GLOBAL even if MV already injected as rate diff might be significant
            if (umv0tile) {
                inside_tile = is_inside_tile_boundary(&(xd->tile),
                    to_inject_mv_x_l0,
                    to_inject_mv_y_l0,
                    mi_col,
                    mi_row,
                    context_ptr->blk_geom->bsize) &&
                    is_inside_tile_boundary(&(xd->tile),
                        to_inject_mv_x_l1,
                        to_inject_mv_y_l1,
                        mi_col,
                        mi_row,
                        context_ptr->blk_geom->bsize);
            }

            inj_mv = inj_mv && inside_tile;

            if (inj_mv && gm_params_0->wmtype > TRANSLATION && gm_params_1->wmtype > TRANSLATION) {
                uint8_t to_inject_ref_type = av1_ref_frame_type(rf);

                // Warped prediction is only compatible with MD_COMP_AVG and MD_COMP_DIST.
                for (cur_type = MD_COMP_AVG;
                    cur_type <= MIN(MD_COMP_DIST, tot_comp_types);
                    cur_type++) {
                    cand_array[cand_total_cnt].type = INTER_MODE;
                    cand_array[cand_total_cnt].distortion_ready = 0;
                    cand_array[cand_total_cnt].use_intrabc = 0;

                    cand_array[cand_total_cnt].merge_flag = EB_FALSE;

                    cand_array[cand_total_cnt].prediction_direction[0] = BI_PRED;

                    cand_array[cand_total_cnt].inter_mode = GLOBAL_GLOBALMV;
                    cand_array[cand_total_cnt].pred_mode = GLOBAL_GLOBALMV;
                    cand_array[cand_total_cnt].motion_mode =
                        gm_params_0->wmtype > TRANSLATION ? WARPED_CAUSAL
                        : SIMPLE_TRANSLATION;
                    cand_array[cand_total_cnt].wm_params_l0 = *gm_params_0;
                    cand_array[cand_total_cnt].wm_params_l1 = *gm_params_1;
                    cand_array[cand_total_cnt].is_compound = 1;
                    cand_array[cand_total_cnt].is_interintra_used = 0;
                    cand_array[cand_total_cnt].is_new_mv = 0;
                    cand_array[cand_total_cnt].drl_index = 0;
                    // will be needed later by the rate estimation
                    cand_array[cand_total_cnt].ref_mv_index = 0;
                    cand_array[cand_total_cnt].ref_frame_type = to_inject_ref_type;
                    cand_array[cand_total_cnt].ref_frame_index_l0 = ref_idx_0;
                    cand_array[cand_total_cnt].ref_frame_index_l1 = ref_idx_1;
                    cand_array[cand_total_cnt].transform_type[0] = DCT_DCT;
                    cand_array[cand_total_cnt].transform_type_uv = DCT_DCT;
                    // Set the MV to frame MV

                    cand_array[cand_total_cnt].motion_vector_xl0 =
                        to_inject_mv_x_l0;
                    cand_array[cand_total_cnt].motion_vector_yl0 =
                        to_inject_mv_y_l0;
                    cand_array[cand_total_cnt].motion_vector_xl1 =
                        to_inject_mv_x_l1;
                    cand_array[cand_total_cnt].motion_vector_yl1 =
                        to_inject_mv_y_l1;
                    //GLOB-GLOB
#if INTER_COMP_REDESIGN
                    if (cur_type == MD_COMP_AVG && tot_comp_types > MD_COMP_AVG)
                        calc_pred_masked_compound(
                            pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);
#if !REMOVE_SIMILARITY_FEATS
                    if (context_ptr->inter_comp_ctrls.similar_predictions)
                        if (cur_type > MD_COMP_AVG &&
                            context_ptr->prediction_mse <=
                            context_ptr->inter_comp_ctrls.similar_predictions_th)
                            continue;
#endif
#endif
                    determine_compound_mode(pcs_ptr,
                        context_ptr,
                        &cand_array[cand_total_cnt],
                        cur_type);
                    INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                }
                context_ptr->injected_mv_x_bipred_l0_array
                    [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                context_ptr->injected_mv_y_bipred_l0_array
                    [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                context_ptr->injected_mv_x_bipred_l1_array
                    [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                context_ptr->injected_mv_y_bipred_l1_array
                    [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                context_ptr->injected_ref_type_bipred_array
                    [context_ptr->injected_mv_count_bipred] =
                    to_inject_ref_type;
                ++context_ptr->injected_mv_count_bipred;
            }
        }
    }
    // update the total number of candidates injected
    (*candidate_total_cnt) = cand_total_cnt;
}
#endif
#if INTER_COMP_REDESIGN
uint8_t is_reference_best_pme(ModeDecisionContext *context_ptr, uint8_t list_index,
    uint8_t ref_index, uint8_t best_x_reference){

    ASSERT(best_x_reference < MAX_NUM_OF_REF_PIC_LIST*REF_LIST_MAX_DEPTH);

    for (int i = 0; i < best_x_reference; i++)
    {
        if (context_ptr->pme_res[0][i].ref_i == ref_index &&
            context_ptr->pme_res[0][i].list_i == list_index)
            return 1;
    }
    return 0;
}
#endif
#if UNIFY_PME_SIGNALS
void inject_pme_candidates(
    //const SequenceControlSet   *scs_ptr,
#else
void inject_predictive_me_candidates(
    //const SequenceControlSet   *scs_ptr,
#endif
    struct ModeDecisionContext *context_ptr, PictureControlSet *pcs_ptr, EbBool is_compound_enabled,
    EbBool allow_bipred, uint32_t *candidate_total_cnt) {
    ModeDecisionCandidate *cand_array      = context_ptr->fast_candidate_array;
    IntMv                  best_pred_mv[2] = {{0}, {0}};
    uint32_t               cand_total_cnt  = (*candidate_total_cnt);
    BlockSize              bsize           = context_ptr->blk_geom->bsize; // bloc size

#if INTER_COMP_REDESIGN
    MD_COMP_TYPE tot_comp_types = context_ptr->compound_types_to_try;
#else
    MD_COMP_TYPE tot_comp_types = (bsize >= BLOCK_8X8 && bsize <= BLOCK_32X32)
                                      ? context_ptr->compound_types_to_try
                                      : context_ptr->compound_types_to_try == MD_COMP_WEDGE
                                            ? MD_COMP_DIFF0
                                            : context_ptr->compound_types_to_try;
#endif
#if !INTER_COMP_REDESIGN
    if (context_ptr->source_variance < context_ptr->inter_inter_wedge_variance_th)
        tot_comp_types = MIN(tot_comp_types, MD_COMP_DIFF0);

#else
#if !OPT_4
    if (context_ptr->source_variance < context_ptr->inter_comp_ctrls.wedge_variance_th)
        tot_comp_types = MIN(tot_comp_types, MD_COMP_DIFF0);
#endif
#endif
#if UPGRADE_SUBPEL
#if INCREASE_WM_CANDS
    Mv mv;
    MvUnit mv_unit;
#endif
    for (uint32_t ref_it = 0; ref_it < pcs_ptr->parent_pcs_ptr->tot_ref_frame_types; ++ref_it) {
        MvReferenceFrame ref_pair = pcs_ptr->parent_pcs_ptr->ref_frame_type_arr[ref_it];
        MvReferenceFrame rf[2];
        av1_set_ref_frame(rf, ref_pair);

        //single ref/list
        if (rf[1] == NONE_FRAME) {
            MvReferenceFrame frame_type = rf[0];
            uint8_t          list_idx = get_list_idx(rf[0]);
            uint8_t          ref_idx = get_ref_frame_idx(rf[0]);

            if (context_ptr->valid_pme_mv[list_idx][ref_idx]) {
                int16_t to_inject_mv_x = context_ptr->best_pme_mv[list_idx][ref_idx][0];
                int16_t to_inject_mv_y = context_ptr->best_pme_mv[list_idx][ref_idx][1];

                uint8_t inj_mv =
                    list_idx == 0
                    ? context_ptr->injected_mv_count_l0 == 0 ||
                    mrp_is_already_injected_mv_l0(
                        context_ptr, to_inject_mv_x, to_inject_mv_y, frame_type) ==
                    EB_FALSE
                    : context_ptr->injected_mv_count_l1 == 0 ||
                    mrp_is_already_injected_mv_l1(
                        context_ptr, to_inject_mv_x, to_inject_mv_y, frame_type) ==
                    EB_FALSE;

                if (inj_mv) {
                    uint8_t inter_type;
                    uint8_t is_ii_allowed = svt_is_interintra_allowed(
                        context_ptr->md_inter_intra_level == 1, bsize, NEWMV, rf);

                    if (context_ptr->md_inter_intra_level > 2)
                        if (is_reference_best_pme(context_ptr, list_idx, ref_idx, 1) == 0)
                            is_ii_allowed = 0;
                    uint8_t tot_inter_types = is_ii_allowed ? II_COUNT : 1;
                    uint8_t is_obmc_allowed =
                        obmc_motion_mode_allowed(
                            pcs_ptr, context_ptr, bsize, rf[0], rf[1], NEWMV) == OBMC_CAUSAL;

                    if (context_ptr->obmc_ctrls.pme_best_ref)
                        if (context_ptr->pme_res[0][0].list_i != list_idx ||
                            context_ptr->pme_res[0][0].ref_i != ref_idx)
                            is_obmc_allowed = 0;
#if INCREASE_WM_CANDS
                    uint8_t is_warp_allowed = warped_motion_mode_allowed(pcs_ptr, context_ptr);
                    tot_inter_types = is_warp_allowed ? tot_inter_types + 1 : tot_inter_types;
#endif
                    tot_inter_types = is_obmc_allowed ? tot_inter_types + 1 : tot_inter_types;
                    for (inter_type = 0; inter_type < tot_inter_types; inter_type++) {
                        cand_array[cand_total_cnt].type = INTER_MODE;
                        cand_array[cand_total_cnt].distortion_ready = 0;
                        cand_array[cand_total_cnt].use_intrabc = 0;
                        cand_array[cand_total_cnt].merge_flag = EB_FALSE;
                        cand_array[cand_total_cnt].prediction_direction[0] = list_idx;
                        cand_array[cand_total_cnt].inter_mode = NEWMV;
                        cand_array[cand_total_cnt].pred_mode = NEWMV;
                        cand_array[cand_total_cnt].motion_mode = SIMPLE_TRANSLATION;
                        cand_array[cand_total_cnt].is_compound = 0;
                        cand_array[cand_total_cnt].is_interintra_used = 0;
                        cand_array[cand_total_cnt].is_new_mv = 1;
                        cand_array[cand_total_cnt].drl_index = 0;
                        if (list_idx == 0) {
                            cand_array[cand_total_cnt].motion_vector_xl0 = to_inject_mv_x;
                            cand_array[cand_total_cnt].motion_vector_yl0 = to_inject_mv_y;
                            context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] =
                                to_inject_mv_x;
                            context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] =
                                to_inject_mv_y;
                            context_ptr
                                ->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] =
                                frame_type;
                            ++context_ptr->injected_mv_count_l0;
                        }
                        else {
                            cand_array[cand_total_cnt].motion_vector_xl1 = to_inject_mv_x;
                            cand_array[cand_total_cnt].motion_vector_yl1 = to_inject_mv_y;
                            context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] =
                                to_inject_mv_x;
                            context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] =
                                to_inject_mv_y;
                            context_ptr
                                ->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] =
                                frame_type;
                            ++context_ptr->injected_mv_count_l1;
                        }
                        cand_array[cand_total_cnt].ref_mv_index = 0;
                        cand_array[cand_total_cnt].ref_frame_type = frame_type;
                        cand_array[cand_total_cnt].ref_frame_index_l0 =
                            (list_idx == 0) ? ref_idx : -1;
                        cand_array[cand_total_cnt].ref_frame_index_l1 =
                            (list_idx == 1) ? ref_idx : -1;
                        cand_array[cand_total_cnt].transform_type[0] = DCT_DCT;
                        cand_array[cand_total_cnt].transform_type_uv = DCT_DCT;

                        choose_best_av1_mv_pred(
                            context_ptr,
                            cand_array[cand_total_cnt].md_rate_estimation_ptr,
                            context_ptr->blk_ptr,
                            cand_array[cand_total_cnt].ref_frame_type,
                            cand_array[cand_total_cnt].is_compound,
                            cand_array[cand_total_cnt].pred_mode,
                            list_idx == 0 ? cand_array[cand_total_cnt].motion_vector_xl0
                            : cand_array[cand_total_cnt].motion_vector_xl1,
                            list_idx == 0 ? cand_array[cand_total_cnt].motion_vector_yl0
                            : cand_array[cand_total_cnt].motion_vector_yl1,
                            0,
                            0,
                            &cand_array[cand_total_cnt].drl_index,
                            best_pred_mv);

                        cand_array[cand_total_cnt].motion_vector_pred_x[list_idx] =
                            best_pred_mv[0].as_mv.col;
                        cand_array[cand_total_cnt].motion_vector_pred_y[list_idx] =
                            best_pred_mv[0].as_mv.row;
                        if (inter_type == 0) {
                            cand_array[cand_total_cnt].is_interintra_used = 0;
                            cand_array[cand_total_cnt].motion_mode = SIMPLE_TRANSLATION;
                        }
                        else {
#if INCREASE_WM_CANDS
                            if (is_warp_allowed && inter_type == (tot_inter_types - (1 + is_obmc_allowed))) {
                                cand_array[cand_total_cnt].is_interintra_used = 0;
                                cand_array[cand_total_cnt].motion_mode = WARPED_CAUSAL;
                                cand_array[cand_total_cnt].wm_params_l0.wmtype = AFFINE;

                                mv.x = to_inject_mv_x;
                                mv.y = to_inject_mv_y;
                                mv_unit.mv[list_idx] = mv;
                                mv_unit.pred_direction = cand_array[cand_total_cnt].prediction_direction[0];
                                cand_array[cand_total_cnt].local_warp_valid = warped_motion_parameters(
                                    pcs_ptr,
                                    context_ptr->blk_ptr,
                                    &mv_unit,
                                    context_ptr->blk_geom,
                                    context_ptr->blk_origin_x,
                                    context_ptr->blk_origin_y,
                                    cand_array[cand_total_cnt].ref_frame_type,
                                    &cand_array[cand_total_cnt].wm_params_l0,
                                    &cand_array[cand_total_cnt].num_proj_ref);
                            }
#endif
                            if (is_obmc_allowed && inter_type == tot_inter_types - 1) {
                                cand_array[cand_total_cnt].is_interintra_used = 0;
                                cand_array[cand_total_cnt].motion_mode = OBMC_CAUSAL;

                                obmc_motion_refinement(
                                    pcs_ptr, context_ptr, &cand_array[cand_total_cnt], list_idx);
                            }
                        }
#if INCREASE_WM_CANDS
                        if (!(is_warp_allowed && inter_type == (tot_inter_types - (1 + is_obmc_allowed))))
                            INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                        else if (cand_array[cand_total_cnt].local_warp_valid)
                            INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
#else
                        INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
#endif
                    }
                }
            }
        }

        else if (is_compound_enabled && allow_bipred) {
            uint8_t ref_idx_0 = get_ref_frame_idx(rf[0]);
            uint8_t ref_idx_1 = get_ref_frame_idx(rf[1]);
            uint8_t list_idx_0 = get_list_idx(rf[0]);
            uint8_t list_idx_1 = get_list_idx(rf[1]);

            if (context_ptr->valid_pme_mv[list_idx_0][ref_idx_0] &&
                context_ptr->valid_pme_mv[list_idx_1][ref_idx_1]) {

                int16_t to_inject_mv_x_l0 =
                    context_ptr->best_pme_mv[list_idx_0][ref_idx_0][0];
                int16_t to_inject_mv_y_l0 =
                    context_ptr->best_pme_mv[list_idx_0][ref_idx_0][1];
                int16_t to_inject_mv_x_l1 =
                    context_ptr->best_pme_mv[list_idx_1][ref_idx_1][0];
                int16_t to_inject_mv_y_l1 =
                    context_ptr->best_pme_mv[list_idx_1][ref_idx_1][1];

                uint8_t to_inject_ref_type = av1_ref_frame_type((const MvReferenceFrame[]){
                    svt_get_ref_frame_type(list_idx_0, ref_idx_0),
                    svt_get_ref_frame_type(list_idx_1, ref_idx_1),
                });
                if (context_ptr->injected_mv_count_bipred == 0 ||
                    mrp_is_already_injected_mv_bipred(context_ptr,
                        to_inject_mv_x_l0,
                        to_inject_mv_y_l0,
                        to_inject_mv_x_l1,
                        to_inject_mv_y_l1,
                        to_inject_ref_type) == EB_FALSE) {
                    if (context_ptr->inter_comp_ctrls.mrp_pruning_w_distortion)

                        if (is_reference_best_pme(context_ptr, list_idx_0, ref_idx_0, 2) == 0 ||
                            is_reference_best_pme(context_ptr, list_idx_1, ref_idx_1, 2) == 0)
                            tot_comp_types = MD_COMP_AVG;

                    for (MD_COMP_TYPE cur_type = MD_COMP_AVG; cur_type <= tot_comp_types; cur_type++) {
#if !REMOVE_SIMILARITY_FEATS
                        // If two predictors are very similar, skip wedge compound mode search
                        if (cur_type == MD_COMP_WEDGE &&
                            context_ptr->inter_comp_ctrls.similar_predictions)
                            if (context_ptr->prediction_mse < 8)
                                continue;
#endif
                        cand_array[cand_total_cnt].type = INTER_MODE;
                        cand_array[cand_total_cnt].distortion_ready = 0;
                        cand_array[cand_total_cnt].use_intrabc = 0;
                        cand_array[cand_total_cnt].merge_flag = EB_FALSE;
                        cand_array[cand_total_cnt].is_new_mv = 1;
                        cand_array[cand_total_cnt].drl_index = 0;
                        // Set the MV to ME result
                        cand_array[cand_total_cnt].motion_vector_xl0 = to_inject_mv_x_l0;
                        cand_array[cand_total_cnt].motion_vector_yl0 = to_inject_mv_y_l0;
                        cand_array[cand_total_cnt].motion_vector_xl1 = to_inject_mv_x_l1;
                        cand_array[cand_total_cnt].motion_vector_yl1 = to_inject_mv_y_l1;
                        // will be needed later by the rate estimation
                        cand_array[cand_total_cnt].ref_mv_index = 0;
                        cand_array[cand_total_cnt].inter_mode = NEW_NEWMV;
                        cand_array[cand_total_cnt].pred_mode = NEW_NEWMV;
                        cand_array[cand_total_cnt].motion_mode = SIMPLE_TRANSLATION;
                        cand_array[cand_total_cnt].is_compound = 1;
                        cand_array[cand_total_cnt].is_interintra_used = 0;
                        cand_array[cand_total_cnt].prediction_direction[0] = BI_PRED;
                        cand_array[cand_total_cnt].ref_frame_type = av1_ref_frame_type((const MvReferenceFrame[]){
                            svt_get_ref_frame_type(list_idx_0, ref_idx_0),
                            svt_get_ref_frame_type(list_idx_1, ref_idx_1),
                        });
                        cand_array[cand_total_cnt].ref_frame_index_l0 = ref_idx_0;
                        cand_array[cand_total_cnt].ref_frame_index_l1 = ref_idx_1;

                        cand_array[cand_total_cnt].transform_type[0] = DCT_DCT;
                        cand_array[cand_total_cnt].transform_type_uv = DCT_DCT;

                        choose_best_av1_mv_pred(context_ptr,
                            cand_array[cand_total_cnt].md_rate_estimation_ptr,
                            context_ptr->blk_ptr,
                            cand_array[cand_total_cnt].ref_frame_type,
                            cand_array[cand_total_cnt].is_compound,
                            cand_array[cand_total_cnt].pred_mode,
                            cand_array[cand_total_cnt].motion_vector_xl0,
                            cand_array[cand_total_cnt].motion_vector_yl0,
                            cand_array[cand_total_cnt].motion_vector_xl1,
                            cand_array[cand_total_cnt].motion_vector_yl1,
                            &cand_array[cand_total_cnt].drl_index,
                            best_pred_mv);

                        cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_0] =
                            best_pred_mv[0].as_mv.col;
                        cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_0] =
                            best_pred_mv[0].as_mv.row;
                        cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_1] =
                            best_pred_mv[1].as_mv.col;
                        cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_1] =
                            best_pred_mv[1].as_mv.row;

                        //MVP REFINE
                        if (cur_type == MD_COMP_AVG && tot_comp_types > MD_COMP_AVG)
                            calc_pred_masked_compound(
                                pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);
#if !REMOVE_SIMILARITY_FEATS
                        if (context_ptr->inter_comp_ctrls.similar_predictions)
                            if (cur_type > MD_COMP_AVG &&
                                context_ptr->prediction_mse <=
                                context_ptr->inter_comp_ctrls.similar_predictions_th)
                                continue;
#endif
                        determine_compound_mode(
                            pcs_ptr, context_ptr, &cand_array[cand_total_cnt], cur_type);
                        INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                        context_ptr
                            ->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_x_l0;
                        context_ptr
                            ->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_y_l0;
                        context_ptr
                            ->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_x_l1;
                        context_ptr
                            ->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_y_l1;

                        context_ptr->injected_ref_type_bipred_array
                            [context_ptr->injected_mv_count_bipred] = to_inject_ref_type;
                        ++context_ptr->injected_mv_count_bipred;
                    }
                }
            }
        }
    }
#else
#if INCREASE_WM_CANDS
    Mv mv_0;
    MvUnit mv_unit;
#endif
    uint8_t list_index;
    uint8_t ref_pic_index;
    list_index = REF_LIST_0;
    {
        // Ref Picture Loop
        for (ref_pic_index = 0; ref_pic_index < 4; ++ref_pic_index) {
            if (context_ptr->valid_refined_mv[list_index][ref_pic_index]) {
                int16_t to_inject_mv_x =
                    context_ptr->best_spatial_pred_mv[list_index][ref_pic_index][0];
                int16_t to_inject_mv_y =
                    context_ptr->best_spatial_pred_mv[list_index][ref_pic_index][1];
                uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_0, ref_pic_index);
                if (context_ptr->injected_mv_count_l0 == 0 ||
                    mrp_is_already_injected_mv_l0(
                        context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) ==
                        EB_FALSE) {
                    uint8_t inter_type;
#if INTRA_COMPOUND_OPT
                    MvReferenceFrame rf[2];
                    rf[0] = to_inject_ref_type;
                    rf[1] = -1;
#endif
                    uint8_t is_ii_allowed =
#if INTRA_COMPOUND_OPT
                        svt_is_interintra_allowed(context_ptr->md_inter_intra_level == 1, bsize, NEWMV, rf);
#else
                        0; // svt_is_interintra_allowed(pcs_ptr->parent_pcs_ptr->enable_inter_intra, bsize, NEWMV, rf);
#endif
#if INTRA_COMPOUND_OPT
                    if (context_ptr->md_inter_intra_level > 2)
                        if (is_reference_best_pme(context_ptr , 0 ,ref_pic_index ,1)  == 0)
                            is_ii_allowed = 0;
#endif

                    uint8_t tot_inter_types = is_ii_allowed ? II_COUNT : 1;
                    uint8_t is_obmc_allowed = obmc_motion_mode_allowed(pcs_ptr,
                                                                       context_ptr,
                                                                       bsize,
                                                                       to_inject_ref_type,
                                                                       -1,
                                                                       NEWMV) == OBMC_CAUSAL;
#if OBMC_FAST
                    if(context_ptr->obmc_ctrls.pme_best_ref)
                        if(context_ptr->pme_res[0][0].list_i!= list_index ||
                           context_ptr->pme_res[0][0].ref_i != ref_pic_index )
                             is_obmc_allowed = 0;
#endif
#if INCREASE_WM_CANDS
                    uint8_t is_warp_allowed = warped_motion_mode_allowed(pcs_ptr, context_ptr);
                    tot_inter_types = is_warp_allowed ? tot_inter_types + 1 : tot_inter_types;
#endif
                    tot_inter_types = is_obmc_allowed ? tot_inter_types + 1 : tot_inter_types;
                    for (inter_type = 0; inter_type < tot_inter_types; inter_type++) {
                        cand_array[cand_total_cnt].type                    = INTER_MODE;
                        cand_array[cand_total_cnt].distortion_ready        = 0;
                        cand_array[cand_total_cnt].use_intrabc             = 0;
                        cand_array[cand_total_cnt].merge_flag              = EB_FALSE;
                        cand_array[cand_total_cnt].prediction_direction[0] = (EbPredDirection)0;
                        cand_array[cand_total_cnt].inter_mode              = NEWMV;
                        cand_array[cand_total_cnt].pred_mode               = NEWMV;
                        cand_array[cand_total_cnt].motion_mode             = SIMPLE_TRANSLATION;
                        cand_array[cand_total_cnt].is_compound             = 0;
                        cand_array[cand_total_cnt].is_interintra_used      = 0;
                        cand_array[cand_total_cnt].is_new_mv               = 1;
                        cand_array[cand_total_cnt].drl_index               = 0;
                        cand_array[cand_total_cnt].motion_vector_xl0       = to_inject_mv_x;
                        cand_array[cand_total_cnt].motion_vector_yl0       = to_inject_mv_y;
                        cand_array[cand_total_cnt].ref_mv_index            = 0;
                        cand_array[cand_total_cnt].ref_frame_type =
                            svt_get_ref_frame_type(REF_LIST_0, ref_pic_index);
                        cand_array[cand_total_cnt].ref_frame_index_l0 = ref_pic_index;
                        cand_array[cand_total_cnt].ref_frame_index_l1 = -1;
                        cand_array[cand_total_cnt].transform_type[0]  = DCT_DCT;
                        cand_array[cand_total_cnt].transform_type_uv  = DCT_DCT;

                        choose_best_av1_mv_pred(context_ptr,
                                                cand_array[cand_total_cnt].md_rate_estimation_ptr,
                                                context_ptr->blk_ptr,
                                                cand_array[cand_total_cnt].ref_frame_type,
                                                cand_array[cand_total_cnt].is_compound,
                                                cand_array[cand_total_cnt].pred_mode,
                                                cand_array[cand_total_cnt].motion_vector_xl0,
                                                cand_array[cand_total_cnt].motion_vector_yl0,
                                                0,
                                                0,
                                                &cand_array[cand_total_cnt].drl_index,
                                                best_pred_mv);

                        cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_0] =
                            best_pred_mv[0].as_mv.col;
                        cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_0] =
                            best_pred_mv[0].as_mv.row;
                        if (inter_type == 0) {
                            cand_array[cand_total_cnt].is_interintra_used = 0;
                            cand_array[cand_total_cnt].motion_mode        = SIMPLE_TRANSLATION;
                        } else {
#if INCREASE_WM_CANDS
                            if (is_warp_allowed && inter_type == (tot_inter_types - (1 + is_obmc_allowed))) {
                                cand_array[cand_total_cnt].is_interintra_used = 0;
                                cand_array[cand_total_cnt].motion_mode = WARPED_CAUSAL;
                                cand_array[cand_total_cnt].wm_params_l0.wmtype = AFFINE;

                                mv.x = to_inject_mv_x;
                                mv.y = to_inject_mv_y;
                                mv_unit.mv[list_index] = mv;
                                mv_unit.pred_direction = cand_array[cand_total_cnt].prediction_direction[0];
                                cand_array[cand_total_cnt].local_warp_valid = warped_motion_parameters(
                                    pcs_ptr,
                                    context_ptr->blk_ptr,
                                    &mv_unit,
                                    context_ptr->blk_geom,
                                    context_ptr->blk_origin_x,
                                    context_ptr->blk_origin_y,
                                    cand_array[cand_total_cnt].ref_frame_type,
                                    &cand_array[cand_total_cnt].wm_params_l0,
                                    &cand_array[cand_total_cnt].num_proj_ref);
                            }
#endif
                            if (is_obmc_allowed && inter_type == tot_inter_types - 1) {
                                cand_array[cand_total_cnt].is_interintra_used = 0;
                                cand_array[cand_total_cnt].motion_mode        = OBMC_CAUSAL;

                                obmc_motion_refinement(
                                    pcs_ptr, context_ptr, &cand_array[cand_total_cnt], REF_LIST_0);
                            }
                        }

#if INCREASE_WM_CANDS
                        if (!(is_warp_allowed && inter_type == (tot_inter_types - (1 + is_obmc_allowed)))) {
                            INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                            context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] =
                                to_inject_mv_x;
                            context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] =
                                to_inject_mv_y;
                            context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] =
                                to_inject_ref_type;
                            ++context_ptr->injected_mv_count_l0;
                        }
                        else if (cand_array[cand_total_cnt].local_warp_valid) {
                            INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                            context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] =
                                to_inject_mv_x;
                            context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] =
                                to_inject_mv_y;
                            context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] =
                                to_inject_ref_type;
                            ++context_ptr->injected_mv_count_l0;
                        }
#else
                        INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                        context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] =
                            to_inject_mv_x;
                        context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] =
                            to_inject_mv_y;
                        context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] =
                            to_inject_ref_type;
                        ++context_ptr->injected_mv_count_l0;
#endif
                    }
                }
            }
        }
    }
    if (is_compound_enabled) {
        /**************
                   NEWMV L1
               ************* */
        list_index = REF_LIST_1;
        {
            // Ref Picture Loop
            for (ref_pic_index = 0; ref_pic_index < 3; ++ref_pic_index) {
                if (context_ptr->valid_refined_mv[list_index][ref_pic_index]) {
                    int16_t to_inject_mv_x =
                        context_ptr->best_spatial_pred_mv[list_index][ref_pic_index][0];
                    int16_t to_inject_mv_y =
                        context_ptr->best_spatial_pred_mv[list_index][ref_pic_index][1];
                    uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_1, ref_pic_index);
                    if (context_ptr->injected_mv_count_l1 == 0 ||
                        mrp_is_already_injected_mv_l1(
                            context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) ==
                            EB_FALSE) {
                        uint8_t inter_type;
#if INTRA_COMPOUND_OPT
                        MvReferenceFrame rf[2];
                        rf[0] = to_inject_ref_type;
                        rf[1] = -1;
#endif
                        uint8_t is_ii_allowed =
#if INTRA_COMPOUND_OPT
                        svt_is_interintra_allowed(context_ptr->md_inter_intra_level == 1, bsize, NEWMV, rf);
#else
                            0; // svt_is_interintra_allowed(pcs_ptr->parent_pcs_ptr->enable_inter_intra, bsize, NEWMV, rf);
#endif
#if INTRA_COMPOUND_OPT
                    if (context_ptr->md_inter_intra_level > 2)
                        if (is_reference_best_pme(context_ptr , 0 ,ref_pic_index ,1)  == 0)
                            is_ii_allowed = 0;
#endif
                        uint8_t tot_inter_types = is_ii_allowed ? II_COUNT : 1;
                        uint8_t is_obmc_allowed = obmc_motion_mode_allowed(pcs_ptr,
                                                                           context_ptr,
                                                                           bsize,
                                                                           to_inject_ref_type,
                                                                           -1,
                                                                           NEWMV) == OBMC_CAUSAL;
#if OBMC_FAST
                        if (context_ptr->obmc_ctrls.pme_best_ref)
                            if (context_ptr->pme_res[0][0].list_i != list_index ||
                                context_ptr->pme_res[0][0].ref_i != ref_pic_index)
                                    is_obmc_allowed = 0;
#endif
#if INCREASE_WM_CANDS
                        uint8_t is_warp_allowed = warped_motion_mode_allowed(pcs_ptr, context_ptr);
                        tot_inter_types = is_warp_allowed ? tot_inter_types + 1 : tot_inter_types;
#endif
                        tot_inter_types = is_obmc_allowed ? tot_inter_types + 1 : tot_inter_types;
                        for (inter_type = 0; inter_type < tot_inter_types; inter_type++) {
                            cand_array[cand_total_cnt].type                    = INTER_MODE;
                            cand_array[cand_total_cnt].distortion_ready        = 0;
                            cand_array[cand_total_cnt].use_intrabc             = 0;
                            cand_array[cand_total_cnt].merge_flag              = EB_FALSE;
                            cand_array[cand_total_cnt].prediction_direction[0] = (EbPredDirection)1;
                            cand_array[cand_total_cnt].inter_mode              = NEWMV;
                            cand_array[cand_total_cnt].pred_mode               = NEWMV;
                            cand_array[cand_total_cnt].motion_mode             = SIMPLE_TRANSLATION;
                            cand_array[cand_total_cnt].is_compound             = 0;
                            cand_array[cand_total_cnt].is_interintra_used      = 0;
                            cand_array[cand_total_cnt].is_new_mv               = 1;
                            cand_array[cand_total_cnt].drl_index               = 0;
                            cand_array[cand_total_cnt].motion_vector_xl1       = to_inject_mv_x;
                            cand_array[cand_total_cnt].motion_vector_yl1       = to_inject_mv_y;
                            cand_array[cand_total_cnt].ref_mv_index            = 0;
                            cand_array[cand_total_cnt].ref_frame_type =
                                svt_get_ref_frame_type(REF_LIST_1, ref_pic_index);
                            cand_array[cand_total_cnt].ref_frame_index_l0 = -1;
                            cand_array[cand_total_cnt].ref_frame_index_l1 = ref_pic_index;
                            cand_array[cand_total_cnt].transform_type[0]  = DCT_DCT;
                            cand_array[cand_total_cnt].transform_type_uv  = DCT_DCT;

                            choose_best_av1_mv_pred(
                                context_ptr,
                                cand_array[cand_total_cnt].md_rate_estimation_ptr,
                                context_ptr->blk_ptr,
                                cand_array[cand_total_cnt].ref_frame_type,
                                cand_array[cand_total_cnt].is_compound,
                                cand_array[cand_total_cnt].pred_mode,
                                cand_array[cand_total_cnt].motion_vector_xl1,
                                cand_array[cand_total_cnt].motion_vector_yl1,
                                0,
                                0,
                                &cand_array[cand_total_cnt].drl_index,
                                best_pred_mv);

                            cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_1] =
                                best_pred_mv[0].as_mv.col;
                            cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_1] =
                                best_pred_mv[0].as_mv.row;
                            if (inter_type == 0) {
                                cand_array[cand_total_cnt].is_interintra_used = 0;
                                cand_array[cand_total_cnt].motion_mode        = SIMPLE_TRANSLATION;
                            } else {
#if INCREASE_WM_CANDS
                                if (is_warp_allowed && inter_type == (tot_inter_types - (1 + is_obmc_allowed))) {
                                    cand_array[cand_total_cnt].is_interintra_used = 0;
                                    cand_array[cand_total_cnt].motion_mode = WARPED_CAUSAL;
                                    cand_array[cand_total_cnt].wm_params_l0.wmtype = AFFINE;

                                    mv.x = to_inject_mv_x;
                                    mv.y = to_inject_mv_y;
                                    mv_unit.mv[list_index] = mv;
                                    mv_unit.pred_direction = cand_array[cand_total_cnt].prediction_direction[0];
                                    cand_array[cand_total_cnt].local_warp_valid = warped_motion_parameters(
                                        pcs_ptr,
                                        context_ptr->blk_ptr,
                                        &mv_unit,
                                        context_ptr->blk_geom,
                                        context_ptr->blk_origin_x,
                                        context_ptr->blk_origin_y,
                                        cand_array[cand_total_cnt].ref_frame_type,
                                        &cand_array[cand_total_cnt].wm_params_l0,
                                        &cand_array[cand_total_cnt].num_proj_ref);
                                }
#endif
                                if (is_obmc_allowed && inter_type == tot_inter_types - 1) {
                                    cand_array[cand_total_cnt].is_interintra_used = 0;
                                    cand_array[cand_total_cnt].motion_mode        = OBMC_CAUSAL;

                                    obmc_motion_refinement(pcs_ptr,
                                                           context_ptr,
                                                           &cand_array[cand_total_cnt],
                                                           REF_LIST_1);
                                }
                            }

#if INCREASE_WM_CANDS
                            if (!(is_warp_allowed && inter_type == (tot_inter_types - (1 + is_obmc_allowed)))) {
                                INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                                context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] =
                                    to_inject_mv_x;
                                context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] =
                                    to_inject_mv_y;
                                context_ptr
                                    ->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] =
                                    to_inject_ref_type;
                                ++context_ptr->injected_mv_count_l1;
                            }
                            else if (cand_array[cand_total_cnt].local_warp_valid) {
                                INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                                context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] =
                                    to_inject_mv_x;
                                context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] =
                                    to_inject_mv_y;
                                context_ptr
                                    ->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] =
                                    to_inject_ref_type;
                                ++context_ptr->injected_mv_count_l1;
                            }
#else
                            INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                            context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] =
                                to_inject_mv_x;
                            context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] =
                                to_inject_mv_y;
                            context_ptr
                                ->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] =
                                to_inject_ref_type;
                            ++context_ptr->injected_mv_count_l1;
#endif
                        }
                    }
                }
            }
        }
    }
    /**************
                NEW_NEWMV
            ************* */
    if (allow_bipred) {
            // Ref Picture Loop
            for (uint8_t ref_pic_index_l0 = 0; ref_pic_index_l0 < 4; ++ref_pic_index_l0) {
                for (uint8_t ref_pic_index_l1 = 0; ref_pic_index_l1 < 4; ++ref_pic_index_l1) {
                    if (context_ptr->valid_refined_mv[REF_LIST_0][ref_pic_index_l0] &&
                        context_ptr->valid_refined_mv[REF_LIST_1][ref_pic_index_l1]) {
                        int16_t to_inject_mv_x_l0 =
                            context_ptr->best_spatial_pred_mv[REF_LIST_0][ref_pic_index_l0][0];
                        int16_t to_inject_mv_y_l0 =
                            context_ptr->best_spatial_pred_mv[REF_LIST_0][ref_pic_index_l0][1];
                        int16_t to_inject_mv_x_l1 =
                            context_ptr->best_spatial_pred_mv[REF_LIST_1][ref_pic_index_l1][0];
                        int16_t to_inject_mv_y_l1 =
                            context_ptr->best_spatial_pred_mv[REF_LIST_1][ref_pic_index_l1][1];

                        uint8_t to_inject_ref_type = av1_ref_frame_type((const MvReferenceFrame[]){
                            svt_get_ref_frame_type(REF_LIST_0, ref_pic_index_l0),
                            svt_get_ref_frame_type(REF_LIST_1, ref_pic_index_l1)});
                        if (context_ptr->injected_mv_count_bipred == 0 ||
                            mrp_is_already_injected_mv_bipred(context_ptr,
                                                              to_inject_mv_x_l0,
                                                              to_inject_mv_y_l0,
                                                              to_inject_mv_x_l1,
                                                              to_inject_mv_y_l1,
                                                              to_inject_ref_type) == EB_FALSE) {
#if !INTER_COMP_REDESIGN
                            context_ptr->variance_ready = 0;
#endif
#if INTER_COMP_REDESIGN
                            if (context_ptr->inter_comp_ctrls.mrp_pruning_w_distortion)
                                if (is_reference_best_pme(context_ptr , 0 ,ref_pic_index_l0 ,2)  == 0 ||
                                     is_reference_best_pme(context_ptr , 1 ,ref_pic_index_l1 ,2) == 0)
                                    tot_comp_types = MD_COMP_AVG;
#endif
                            for (MD_COMP_TYPE cur_type = MD_COMP_AVG; cur_type <= tot_comp_types; cur_type++) {
                                // If two predictors are very similar, skip wedge compound mode search
#if INTER_COMP_REDESIGN
                                if (cur_type == MD_COMP_WEDGE && context_ptr->inter_comp_ctrls.similar_predictions)
#else
                                if (context_ptr->variance_ready)
#endif
                                    if (context_ptr->prediction_mse < 8)
                                        continue;

                                cand_array[cand_total_cnt].type             = INTER_MODE;
                                cand_array[cand_total_cnt].distortion_ready = 0;
                                cand_array[cand_total_cnt].use_intrabc      = 0;
                                cand_array[cand_total_cnt].merge_flag       = EB_FALSE;
                                cand_array[cand_total_cnt].is_new_mv        = 1;
                                cand_array[cand_total_cnt].drl_index        = 0;
                                // Set the MV to ME result
                                cand_array[cand_total_cnt].motion_vector_xl0 = to_inject_mv_x_l0;
                                cand_array[cand_total_cnt].motion_vector_yl0 = to_inject_mv_y_l0;
                                cand_array[cand_total_cnt].motion_vector_xl1 = to_inject_mv_x_l1;
                                cand_array[cand_total_cnt].motion_vector_yl1 = to_inject_mv_y_l1;
                                // will be needed later by the rate estimation
                                cand_array[cand_total_cnt].ref_mv_index       = 0;
                                cand_array[cand_total_cnt].inter_mode         = NEW_NEWMV;
                                cand_array[cand_total_cnt].pred_mode          = NEW_NEWMV;
                                cand_array[cand_total_cnt].motion_mode        = SIMPLE_TRANSLATION;
                                cand_array[cand_total_cnt].is_compound        = 1;
                                cand_array[cand_total_cnt].is_interintra_used = 0;
                                cand_array[cand_total_cnt].prediction_direction[0] =
                                    (EbPredDirection)2;

                                cand_array[cand_total_cnt].ref_frame_type = av1_ref_frame_type(
                                    (const MvReferenceFrame[]){
                                        svt_get_ref_frame_type(REF_LIST_0, ref_pic_index_l0),
                                        svt_get_ref_frame_type(REF_LIST_1, ref_pic_index_l1)});
                                cand_array[cand_total_cnt].ref_frame_index_l0 = ref_pic_index_l0;
                                cand_array[cand_total_cnt].ref_frame_index_l1 = ref_pic_index_l1;

                                cand_array[cand_total_cnt].transform_type[0] = DCT_DCT;
                                cand_array[cand_total_cnt].transform_type_uv = DCT_DCT;

                                choose_best_av1_mv_pred(
                                    context_ptr,
                                    cand_array[cand_total_cnt].md_rate_estimation_ptr,
                                    context_ptr->blk_ptr,
                                    cand_array[cand_total_cnt].ref_frame_type,
                                    cand_array[cand_total_cnt].is_compound,
                                    cand_array[cand_total_cnt].pred_mode,
                                    cand_array[cand_total_cnt].motion_vector_xl0,
                                    cand_array[cand_total_cnt].motion_vector_yl0,
                                    cand_array[cand_total_cnt].motion_vector_xl1,
                                    cand_array[cand_total_cnt].motion_vector_yl1,
                                    &cand_array[cand_total_cnt].drl_index,
                                    best_pred_mv);
                                cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_0] =
                                    best_pred_mv[0].as_mv.col;
                                cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_0] =
                                    best_pred_mv[0].as_mv.row;
                                cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_1] =
                                    best_pred_mv[1].as_mv.col;
                                cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_1] =
                                    best_pred_mv[1].as_mv.row;

                                //MVP REFINE
#if INTER_COMP_REDESIGN
                                if (cur_type == MD_COMP_AVG && tot_comp_types > MD_COMP_AVG)
                                    calc_pred_masked_compound(
                                        pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);

                                if (context_ptr->inter_comp_ctrls.similar_predictions)
                                    if (cur_type > MD_COMP_AVG &&
                                        context_ptr->prediction_mse <=
                                        context_ptr->inter_comp_ctrls.similar_predictions_th )
                                        continue;

#endif
                                determine_compound_mode(
                                    pcs_ptr, context_ptr, &cand_array[cand_total_cnt], cur_type);
                                INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                                context_ptr->injected_mv_x_bipred_l0_array
                                    [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                                context_ptr->injected_mv_y_bipred_l0_array
                                    [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                                context_ptr->injected_mv_x_bipred_l1_array
                                    [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                                context_ptr->injected_mv_y_bipred_l1_array
                                    [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;

                                context_ptr->injected_ref_type_bipred_array
                                    [context_ptr->injected_mv_count_bipred] = to_inject_ref_type;
                                ++context_ptr->injected_mv_count_bipred;
                            }
                        }
                    }
                }
            }
    }
#endif
    (*candidate_total_cnt) = cand_total_cnt;
}

void inject_inter_candidates(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
#if SWITCH_MODE_BASED_ON_SQ_COEFF
                             const SequenceControlSet *scs_ptr, SuperBlock *sb_ptr, uint32_t *candidate_total_cnt) {
#else
                             const SequenceControlSet *scs_ptr, SuperBlock *sb_ptr, EbBool coeff_based_nsq_cand_reduction, uint32_t *candidate_total_cnt) {
#endif
    (void)scs_ptr;

    FrameHeader *          frm_hdr        = &pcs_ptr->parent_pcs_ptr->frm_hdr;
    uint32_t               cand_total_cnt = *candidate_total_cnt;
#if !IMPROVE_GMV
    ModeDecisionCandidate *cand_array     = context_ptr->fast_candidate_array;
#endif
    EbBool       is_compound_enabled      = (frm_hdr->reference_mode == SINGLE_REFERENCE) ? 0 : 1;
#if !IMPROVE_GMV
    MacroBlockD *xd                       = context_ptr->blk_ptr->av1xd;
    int          umv0tile = (scs_ptr->static_config.unrestricted_motion_vector == 0);
#endif
    MeSbResults *me_results =
        pcs_ptr->parent_pcs_ptr->pa_me_data->me_results[context_ptr->me_sb_addr];
#if FIRST_PASS_SETUP
    EbBool       allow_bipred =
        (use_output_stat(scs_ptr) || context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4)
        ? EB_FALSE : EB_TRUE;
#else
    EbBool allow_bipred = context_ptr->blk_geom->bwidth != 4 && context_ptr->blk_geom->bheight != 4;
#endif
#if !IMPROVE_GMV
    BlockSize bsize     = context_ptr->blk_geom->bsize; // bloc size

    MD_COMP_TYPE tot_comp_types = context_ptr->compound_types_to_try;
    if (context_ptr->source_variance < context_ptr->inter_comp_ctrls.wedge_variance_th)
        tot_comp_types = MIN(tot_comp_types, MD_COMP_DIFF0);
#endif
    uint32_t mi_row = context_ptr->blk_origin_y >> MI_SIZE_LOG2;
    uint32_t mi_col = context_ptr->blk_origin_x >> MI_SIZE_LOG2;
    eb_av1_count_overlappable_neighbors(
        pcs_ptr, context_ptr->blk_ptr, context_ptr->blk_geom->bsize, mi_row, mi_col);
    const uint8_t is_obmc_allowed = obmc_motion_mode_allowed(pcs_ptr,
                                                             context_ptr,
                                                             context_ptr->blk_geom->bsize,
                                                             LAST_FRAME,
                                                             -1,
                                                             NEWMV) == OBMC_CAUSAL;
    if (is_obmc_allowed)
        precompute_obmc_data(pcs_ptr, context_ptr);
    /**************
         MVP
    ************* */
    if (context_ptr->new_nearest_injection)
        //all of ref pairs: (1)single-ref List0  (2)single-ref List1  (3)compound Bi-Dir List0-List1  (4)compound Uni-Dir List0-List0  (5)compound Uni-Dir List1-List1
        for (uint32_t ref_it = 0; ref_it < pcs_ptr->parent_pcs_ptr->tot_ref_frame_types; ++ref_it) {
            const MvReferenceFrame ref_frame_pair =
                pcs_ptr->parent_pcs_ptr->ref_frame_type_arr[ref_it];
            inject_mvp_candidates_ii(
                context_ptr, pcs_ptr, context_ptr->blk_ptr, ref_frame_pair, &cand_total_cnt);
        }

    //----------------------
    //    NEAREST_NEWMV, NEW_NEARESTMV, NEAR_NEWMV, NEW_NEARMV.
    //----------------------
    if (context_ptr->new_nearest_near_comb_injection) {
        const EbBool allow_compound = frm_hdr->reference_mode != SINGLE_REFERENCE &&
            context_ptr->blk_geom->bwidth != 4 && context_ptr->blk_geom->bheight != 4;
        if (allow_compound) {
            //all of ref pairs: (1)single-ref List0  (2)single-ref List1  (3)compound Bi-Dir List0-List1  (4)compound Uni-Dir List0-List0  (5)compound Uni-Dir List1-List1
            for (uint32_t ref_it = 0; ref_it < pcs_ptr->parent_pcs_ptr->tot_ref_frame_types;
                 ++ref_it) {
                const MvReferenceFrame ref_frame_pair =
                    pcs_ptr->parent_pcs_ptr->ref_frame_type_arr[ref_it];
                inject_new_nearest_new_comb_candidates(
                    scs_ptr, context_ptr, pcs_ptr, ref_frame_pair, &cand_total_cnt);
            }
        }
    }
    inject_new_candidates(scs_ptr,
                          context_ptr,
                          pcs_ptr,
                          is_compound_enabled,
                          allow_bipred,
                          context_ptr->me_sb_addr,
                          context_ptr->me_block_offset,
                          &cand_total_cnt);
    if (context_ptr->global_mv_injection) {
#if IMPROVE_GMV
        inject_global_candidates(scs_ptr,
            context_ptr,
            pcs_ptr,
            is_compound_enabled,
            allow_bipred,
            &cand_total_cnt);
#else
        if (pcs_ptr->parent_pcs_ptr->gm_level <= GM_DOWN16) {
            for (unsigned list_ref_index_l0 = 0; list_ref_index_l0 < 1; ++list_ref_index_l0)
                for (unsigned list_ref_index_l1 = 0; list_ref_index_l1 < 1; ++list_ref_index_l1) {
                    /**************
                     GLOBALMV
                    ************* */
                    uint8_t               to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_0,
                                                                        list_ref_index_l0);
                    EbWarpedMotionParams *params_l0 =
                        &pcs_ptr->parent_pcs_ptr->global_motion[to_inject_ref_type];

                    IntMv mv_l0 = gm_get_motion_vector_enc(
                        params_l0,
                        pcs_ptr->parent_pcs_ptr->frm_hdr.allow_high_precision_mv,
                        context_ptr->blk_geom->bsize,
                        mi_col,
                        mi_row,
                        0 /* force_integer_mv */);

                    int16_t to_inject_mv_x_l0 = mv_l0.as_mv.col;
                    int16_t to_inject_mv_y_l0 = mv_l0.as_mv.row;

                    int inside_tile = umv0tile
                        ? is_inside_tile_boundary(&(xd->tile),
                                                  to_inject_mv_x_l0,
                                                  to_inject_mv_y_l0,
                                                  mi_col,
                                                  mi_row,
                                                  context_ptr->blk_geom->bsize)
                        : 1;

                    if (inside_tile &&
                        (((params_l0->wmtype > TRANSLATION && context_ptr->blk_geom->bwidth >= 8 &&
                           context_ptr->blk_geom->bheight >= 8) ||
                          params_l0->wmtype <= TRANSLATION))) {
                        int is_ii_allowed = svt_is_interintra_allowed(
                            context_ptr->md_inter_intra_level,
                            bsize,
                            GLOBALMV,
                            (const MvReferenceFrame[]){to_inject_ref_type, -1});
                        int tot_inter_types = is_ii_allowed ? II_COUNT : 1;
                        //uint8_t is_obmc_allowed =  obmc_motion_mode_allowed(pcs_ptr, context_ptr->blk_ptr, bsize, rf[0], rf[1], NEWMV) == OBMC_CAUSAL;
                        //tot_inter_types = is_obmc_allowed ? tot_inter_types+1 : tot_inter_types;

                        for (int inter_type = 0; inter_type < tot_inter_types; inter_type++) {
                            cand_array[cand_total_cnt].type = INTER_MODE;

                            cand_array[cand_total_cnt].distortion_ready = 0;
                            cand_array[cand_total_cnt].use_intrabc      = 0;

                            cand_array[cand_total_cnt].merge_flag = EB_FALSE;

                            cand_array[cand_total_cnt].inter_mode  = GLOBALMV;
                            cand_array[cand_total_cnt].pred_mode   = GLOBALMV;
                            cand_array[cand_total_cnt].motion_mode = params_l0->wmtype > TRANSLATION
                                ? WARPED_CAUSAL
                                : SIMPLE_TRANSLATION;

                            cand_array[cand_total_cnt].wm_params_l0 = *params_l0;

                            cand_array[cand_total_cnt].is_compound             = 0;
                            cand_array[cand_total_cnt].distortion_ready        = 0;
                            cand_array[cand_total_cnt].use_intrabc             = 0;
                            cand_array[cand_total_cnt].merge_flag              = EB_FALSE;
                            cand_array[cand_total_cnt].prediction_direction[0] = UNI_PRED_LIST_0;
                            cand_array[cand_total_cnt].is_new_mv               = 0;
                            cand_array[cand_total_cnt].motion_vector_xl0       = to_inject_mv_x_l0;
                            cand_array[cand_total_cnt].motion_vector_yl0       = to_inject_mv_y_l0;
                            cand_array[cand_total_cnt].drl_index               = 0;
                            cand_array[cand_total_cnt].ref_mv_index            = 0;
                            cand_array[cand_total_cnt].ref_frame_type          = av1_ref_frame_type(
                                (const MvReferenceFrame[]){to_inject_ref_type, -1});
                            cand_array[cand_total_cnt].ref_frame_index_l0 = 0;
                            cand_array[cand_total_cnt].ref_frame_index_l1 = -1;
                            cand_array[cand_total_cnt].transform_type[0]  = DCT_DCT;
                            cand_array[cand_total_cnt].transform_type_uv  = DCT_DCT;
                            if (inter_type == 0) {
                                cand_array[cand_total_cnt].is_interintra_used = 0;
                            } else {
                                if (is_ii_allowed) {
                                    if (inter_type == 1) {
                                        inter_intra_search(
                                            pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);
                                        cand_array[cand_total_cnt].is_interintra_used   = 1;
                                        cand_array[cand_total_cnt].use_wedge_interintra = 1;
                                    } else if (inter_type == 2) {
                                        cand_array[cand_total_cnt].is_interintra_used = 1;
                                        cand_array[cand_total_cnt].interintra_mode =
                                            cand_array[cand_total_cnt - 1].interintra_mode;
                                        cand_array[cand_total_cnt].use_wedge_interintra = 0;
                                    }
                                }
                                //if (is_obmc_allowed && inter_type == tot_inter_types - 1) {
                                //    cand_array[cand_total_cnt].is_interintra_used = 0;
                                //    cand_array[cand_total_cnt].motion_mode = OBMC_CAUSAL;
                                //}
                            }
                            INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                        }

                        context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] =
                            to_inject_mv_x_l0;
                        context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] =
                            to_inject_mv_y_l0;
                        context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] =
                            to_inject_ref_type;
                        ++context_ptr->injected_mv_count_l0;

                        EbWarpedMotionParams *params_l1 =
                            &pcs_ptr->parent_pcs_ptr->global_motion[svt_get_ref_frame_type(
                                REF_LIST_1, list_ref_index_l1)];

                        if (is_compound_enabled && allow_bipred &&
                            (params_l0->wmtype > TRANSLATION && params_l1->wmtype > TRANSLATION)) {
                            /**************
                            GLOBAL_GLOBALMV
                            ************* */

                            IntMv mv_l1 = gm_get_motion_vector_enc(
                                params_l1,
                                pcs_ptr->parent_pcs_ptr->frm_hdr.allow_high_precision_mv,
                                context_ptr->blk_geom->bsize,
                                mi_col,
                                mi_row,
                                0 /* force_integer_mv */);

                            int16_t to_inject_mv_x_l1 = mv_l1.as_mv.col;
                            int16_t to_inject_mv_y_l1 = mv_l1.as_mv.row;

                            if (!umv0tile ||
                                is_inside_tile_boundary(&(xd->tile),
                                                        to_inject_mv_x_l0,
                                                        to_inject_mv_y_l1,
                                                        mi_col,
                                                        mi_row,
                                                        context_ptr->blk_geom->bsize)) {
                                to_inject_ref_type = av1_ref_frame_type((const MvReferenceFrame[]){
                                    svt_get_ref_frame_type(REF_LIST_0, list_ref_index_l0),
                                    svt_get_ref_frame_type(REF_LIST_1, list_ref_index_l1)});

                                // Warped prediction is only compatible with MD_COMP_AVG and MD_COMP_DIST.
                                for (MD_COMP_TYPE cur_type = MD_COMP_AVG;
                                     cur_type <= MIN(MD_COMP_DIST, tot_comp_types);
                                     cur_type++) {
                                    cand_array[cand_total_cnt].type             = INTER_MODE;
                                    cand_array[cand_total_cnt].distortion_ready = 0;
                                    cand_array[cand_total_cnt].use_intrabc      = 0;

                                    cand_array[cand_total_cnt].merge_flag = EB_FALSE;

                                    cand_array[cand_total_cnt].prediction_direction[0] =
                                        (EbPredDirection)2;

                                    cand_array[cand_total_cnt].inter_mode  = GLOBAL_GLOBALMV;
                                    cand_array[cand_total_cnt].pred_mode   = GLOBAL_GLOBALMV;
                                    cand_array[cand_total_cnt].motion_mode = params_l0->wmtype >
                                            TRANSLATION
                                        ? WARPED_CAUSAL
                                        : SIMPLE_TRANSLATION;
                                    cand_array[cand_total_cnt].wm_params_l0       = *params_l0;
                                    cand_array[cand_total_cnt].wm_params_l1       = *params_l1;
                                    cand_array[cand_total_cnt].is_compound        = 1;
                                    cand_array[cand_total_cnt].is_interintra_used = 0;
                                    cand_array[cand_total_cnt].is_new_mv          = 0;
                                    cand_array[cand_total_cnt].drl_index          = 0;
                                    // will be needed later by the rate estimation
                                    cand_array[cand_total_cnt].ref_mv_index   = 0;
                                    cand_array[cand_total_cnt].ref_frame_type = to_inject_ref_type;
                                    cand_array[cand_total_cnt].ref_frame_index_l0 = 0;
                                    cand_array[cand_total_cnt].ref_frame_index_l1 = 0;
                                    cand_array[cand_total_cnt].transform_type[0]  = DCT_DCT;
                                    cand_array[cand_total_cnt].transform_type_uv  = DCT_DCT;
                                    // Set the MV to frame MV

                                    cand_array[cand_total_cnt].motion_vector_xl0 =
                                        to_inject_mv_x_l0;
                                    cand_array[cand_total_cnt].motion_vector_yl0 =
                                        to_inject_mv_y_l0;
                                    cand_array[cand_total_cnt].motion_vector_xl1 =
                                        to_inject_mv_x_l1;
                                    cand_array[cand_total_cnt].motion_vector_yl1 =
                                        to_inject_mv_y_l1;
                                    //GLOB-GLOB
                                    if (cur_type == MD_COMP_AVG && tot_comp_types > MD_COMP_AVG)
                                        calc_pred_masked_compound(
                                            pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);

                                    if (context_ptr->inter_comp_ctrls.similar_predictions &&
                                        cur_type > MD_COMP_AVG &&
                                        context_ptr->prediction_mse <=
                                            context_ptr->inter_comp_ctrls.similar_predictions_th)
                                        continue;

                                    determine_compound_mode(pcs_ptr,
                                                            context_ptr,
                                                            &cand_array[cand_total_cnt],
                                                            cur_type);
                                    INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);

                                    context_ptr->injected_mv_x_bipred_l0_array
                                        [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                                    context_ptr->injected_mv_y_bipred_l0_array
                                        [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                                    context_ptr->injected_mv_x_bipred_l1_array
                                        [context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                                    context_ptr->injected_mv_y_bipred_l1_array
                                        [context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                                    context_ptr->injected_ref_type_bipred_array
                                        [context_ptr->injected_mv_count_bipred] =
                                        to_inject_ref_type;
                                    ++context_ptr->injected_mv_count_bipred;
                                }
                            }
                        }
                    }
                }
        } else {
            /**************
             GLOBALMV L0
            ************* */
            int16_t to_inject_mv_x = (int16_t)(
                pcs_ptr->parent_pcs_ptr->global_motion[LAST_FRAME].wmmat[1] >>
                GM_TRANS_ONLY_PREC_DIFF);
            int16_t to_inject_mv_y = (int16_t)(
                pcs_ptr->parent_pcs_ptr->global_motion[LAST_FRAME].wmmat[0] >>
                GM_TRANS_ONLY_PREC_DIFF);
            uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_0, 0 /*list0_ref_index*/);
            const uint8_t inj_mv       = context_ptr->injected_mv_count_l0 == 0 ||
                mrp_is_already_injected_mv_l0(
                    context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE;
            const int inside_tile = umv0tile ? is_inside_tile_boundary(&(xd->tile),
                                                                       to_inject_mv_x,
                                                                       to_inject_mv_y,
                                                                       mi_col,
                                                                       mi_row,
                                                                       context_ptr->blk_geom->bsize)
                                             : 1;
            if (inj_mv && inside_tile) {
                uint8_t is_ii_allowed = svt_is_interintra_allowed(
                    context_ptr->md_inter_intra_level,
                    bsize,
                    GLOBALMV,
                    (const MvReferenceFrame[]){to_inject_ref_type, -1});
                uint8_t tot_inter_types = is_ii_allowed ? II_COUNT : 1;
                //uint8_t is_obmc_allowed =  obmc_motion_mode_allowed(pcs_ptr, context_ptr->blk_ptr, bsize, rf[0], rf[1], NEWMV) == OBMC_CAUSAL;
                //tot_inter_types = is_obmc_allowed ? tot_inter_types+1 : tot_inter_types;

                for (uint8_t inter_type = 0; inter_type < tot_inter_types; inter_type++) {
                    cand_array[cand_total_cnt].type = INTER_MODE;

                    cand_array[cand_total_cnt].distortion_ready = 0;
                    cand_array[cand_total_cnt].use_intrabc      = 0;

                    cand_array[cand_total_cnt].merge_flag              = EB_FALSE;
                    cand_array[cand_total_cnt].prediction_direction[0] = (EbPredDirection)0;

                    cand_array[cand_total_cnt].inter_mode  = GLOBALMV;
                    cand_array[cand_total_cnt].pred_mode   = GLOBALMV;
                    cand_array[cand_total_cnt].motion_mode = SIMPLE_TRANSLATION;
                    cand_array[cand_total_cnt].is_compound = 0;
                    cand_array[cand_total_cnt].is_new_mv   = 0;
                    cand_array[cand_total_cnt].drl_index   = 0;

                    // will be needed later by the rate estimation
                    cand_array[cand_total_cnt].ref_mv_index       = 0;
                    cand_array[cand_total_cnt].ref_frame_type     = LAST_FRAME;
                    cand_array[cand_total_cnt].ref_frame_index_l0 = 0;
                    cand_array[cand_total_cnt].ref_frame_index_l1 = -1;

                    cand_array[cand_total_cnt].transform_type[0] = DCT_DCT;
                    cand_array[cand_total_cnt].transform_type_uv = DCT_DCT;
                    // Set the MV to frame MV
                    cand_array[cand_total_cnt].motion_vector_xl0 = to_inject_mv_x;
                    cand_array[cand_total_cnt].motion_vector_yl0 = to_inject_mv_y;
                    if (inter_type == 0) {
                        cand_array[cand_total_cnt].is_interintra_used = 0;
                        cand_array[cand_total_cnt].motion_mode        = SIMPLE_TRANSLATION;
                    } else {
                        if (is_ii_allowed) {
                            if (inter_type == 1) {
                                inter_intra_search(
                                    pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);
                                cand_array[cand_total_cnt].is_interintra_used   = 1;
                                cand_array[cand_total_cnt].use_wedge_interintra = 1;
                            } else if (inter_type == 2) {
                                cand_array[cand_total_cnt].is_interintra_used = 1;
                                cand_array[cand_total_cnt].interintra_mode =
                                    cand_array[cand_total_cnt - 1].interintra_mode;
                                cand_array[cand_total_cnt].use_wedge_interintra = 0;
                            }
                        }
                    }
                    INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                }
                context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] =
                    to_inject_mv_x;
                context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] =
                    to_inject_mv_y;
                context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] =
                    to_inject_ref_type;
                ++context_ptr->injected_mv_count_l0;
            }

            if (is_compound_enabled && allow_bipred) {
                /**************
            GLOBAL_GLOBALMV
            ************* */

                int16_t to_inject_mv_x_l0 = (int16_t)(
                    pcs_ptr->parent_pcs_ptr->global_motion[LAST_FRAME].wmmat[1] >>
                    GM_TRANS_ONLY_PREC_DIFF);
                int16_t to_inject_mv_y_l0 = (int16_t)(
                    pcs_ptr->parent_pcs_ptr->global_motion[LAST_FRAME].wmmat[0] >>
                    GM_TRANS_ONLY_PREC_DIFF);
                int16_t to_inject_mv_x_l1 = (int16_t)(
                    pcs_ptr->parent_pcs_ptr->global_motion[BWDREF_FRAME].wmmat[1] >>
                    GM_TRANS_ONLY_PREC_DIFF);
                int16_t to_inject_mv_y_l1 = (int16_t)(
                    pcs_ptr->parent_pcs_ptr->global_motion[BWDREF_FRAME].wmmat[0] >>
                    GM_TRANS_ONLY_PREC_DIFF);
                to_inject_ref_type = av1_ref_frame_type((const MvReferenceFrame[]){
                    svt_get_ref_frame_type(REF_LIST_0, 0 /*list0_ref_index*/),
                    svt_get_ref_frame_type(REF_LIST_1, 0 /*list1_ref_index*/)});

                if ((!umv0tile ||
                     is_inside_tile_boundary(&(xd->tile),
                                             to_inject_mv_x_l0,
                                             to_inject_mv_y_l1,
                                             mi_col,
                                             mi_row,
                                             context_ptr->blk_geom->bsize)) &&
                    (context_ptr->injected_mv_count_bipred == 0 ||
                     mrp_is_already_injected_mv_bipred(context_ptr,
                                                       to_inject_mv_x_l0,
                                                       to_inject_mv_y_l0,
                                                       to_inject_mv_x_l1,
                                                       to_inject_mv_y_l1,
                                                       to_inject_ref_type) == EB_FALSE)) {
                    for (MD_COMP_TYPE cur_type = MD_COMP_AVG; cur_type <= tot_comp_types;
                         cur_type++) {
                        // If two predictors are very similar, skip wedge compound mode search
                        if (cur_type == MD_COMP_WEDGE &&
                            context_ptr->inter_comp_ctrls.similar_predictions &&
                            context_ptr->prediction_mse < 64)
                            continue;
                        cand_array[cand_total_cnt].type                    = INTER_MODE;
                        cand_array[cand_total_cnt].distortion_ready        = 0;
                        cand_array[cand_total_cnt].use_intrabc             = 0;
                        cand_array[cand_total_cnt].merge_flag              = EB_FALSE;
                        cand_array[cand_total_cnt].prediction_direction[0] = (EbPredDirection)2;
                        cand_array[cand_total_cnt].inter_mode              = GLOBAL_GLOBALMV;
                        cand_array[cand_total_cnt].pred_mode               = GLOBAL_GLOBALMV;
                        cand_array[cand_total_cnt].motion_mode             = SIMPLE_TRANSLATION;
                        cand_array[cand_total_cnt].is_compound             = 1;
                        cand_array[cand_total_cnt].is_interintra_used      = 0;
                        cand_array[cand_total_cnt].is_new_mv               = 0;
                        cand_array[cand_total_cnt].drl_index               = 0;

                        // will be needed later by the rate estimation
                        cand_array[cand_total_cnt].ref_mv_index       = 0;
                        cand_array[cand_total_cnt].ref_frame_type     = LAST_BWD_FRAME;
                        cand_array[cand_total_cnt].ref_frame_index_l0 = 0;
                        cand_array[cand_total_cnt].ref_frame_index_l1 = 0;
                        cand_array[cand_total_cnt].transform_type[0]  = DCT_DCT;
                        cand_array[cand_total_cnt].transform_type_uv  = DCT_DCT;
                        // Set the MV to frame MV

                        cand_array[cand_total_cnt].motion_vector_xl0 = to_inject_mv_x_l0;
                        cand_array[cand_total_cnt].motion_vector_yl0 = to_inject_mv_y_l0;
                        cand_array[cand_total_cnt].motion_vector_xl1 = to_inject_mv_x_l1;
                        cand_array[cand_total_cnt].motion_vector_yl1 = to_inject_mv_y_l1;
                        //GLOB-GLOB
                        if (cur_type == MD_COMP_AVG && tot_comp_types > MD_COMP_AVG)
                            calc_pred_masked_compound(
                                pcs_ptr, context_ptr, &cand_array[cand_total_cnt]);

                        if (context_ptr->inter_comp_ctrls.similar_predictions &&
                            cur_type > MD_COMP_AVG &&
                            context_ptr->prediction_mse <=
                                context_ptr->inter_comp_ctrls.similar_predictions_th)
                            continue;
                        determine_compound_mode(
                            pcs_ptr, context_ptr, &cand_array[cand_total_cnt], cur_type);
                        INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);

                        context_ptr
                            ->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_x_l0;
                        context_ptr
                            ->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_y_l0;
                        context_ptr
                            ->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_x_l1;
                        context_ptr
                            ->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] =
                            to_inject_mv_y_l1;
                        context_ptr->injected_ref_type_bipred_array
                            [context_ptr->injected_mv_count_bipred] = to_inject_ref_type;
                        ++context_ptr->injected_mv_count_bipred;
                    }
                }
            }
        }
#endif
    }

    // Warped Motion
#if INCREASE_WM_CANDS
    if (warped_motion_mode_allowed(pcs_ptr, context_ptr)) {
#else
    if (frm_hdr->allow_warped_motion && has_overlappable_candidates(context_ptr->blk_ptr) &&
        context_ptr->blk_geom->bwidth >= 8 && context_ptr->blk_geom->bheight >= 8 &&
        context_ptr->warped_motion_injection) {
#endif
        inject_warped_motion_candidates(
            pcs_ptr, context_ptr, context_ptr->blk_ptr, &cand_total_cnt, me_results);
    }
#if SWITCH_MODE_BASED_ON_SQ_COEFF
    if (is_compound_enabled) {
#else
    if (!coeff_based_nsq_cand_reduction && is_compound_enabled) {
#endif
        if (allow_bipred && context_ptr->bipred3x3_injection > 0 && pcs_ptr->slice_type == B_SLICE)
            //----------------------
            // Bipred2Nx2N
            //----------------------
            bipred_3x3_candidates_injection(
                scs_ptr, pcs_ptr, context_ptr, sb_ptr, context_ptr->me_sb_addr, &cand_total_cnt);

        //----------------------
        // Unipred2Nx2N
        //----------------------
        if (context_ptr->unipred3x3_injection > 0 && pcs_ptr->slice_type != I_SLICE)
            unipred_3x3_candidates_injection(
                scs_ptr, pcs_ptr, context_ptr, sb_ptr, context_ptr->me_sb_addr, &cand_total_cnt);
    }
#if UNIFY_PME_SIGNALS
    if (context_ptr->md_pme_ctrls.enabled)
        inject_pme_candidates(
#else
    if (context_ptr->predictive_me_level)
        inject_predictive_me_candidates(
#endif
            context_ptr, pcs_ptr, is_compound_enabled, allow_bipred, &cand_total_cnt);
    // update the total number of candidates injected
    *candidate_total_cnt = cand_total_cnt;
}

static INLINE TxType av1_get_tx_type(BlockSize sb_type, int32_t is_inter, PredictionMode pred_mode,
                                     UvPredictionMode pred_mode_uv, PlaneType plane_type,
                                     const MacroBlockD *xd, int32_t blk_row, int32_t blk_col,
                                     TxSize tx_size, int32_t reduced_tx_set) {
    UNUSED(sb_type);
    UNUSED(xd);
    UNUSED(blk_row);
    UNUSED(blk_col);

    // BlockSize  sb_type = BLOCK_8X8;

    MbModeInfo mbmi;
    mbmi.block_mi.mode    = pred_mode;
    mbmi.block_mi.uv_mode = pred_mode_uv;

    // const MbModeInfo *const mbmi = xd->mi[0];
    // const struct MacroblockdPlane *const pd = &xd->plane[plane_type];
    const TxSetType tx_set_type =
        /*av1_*/ get_ext_tx_set_type(tx_size, is_inter, reduced_tx_set);

    TxType tx_type = DCT_DCT;
    if (/*xd->lossless[mbmi->segment_id] ||*/ txsize_sqr_up_map[tx_size] > TX_32X32)
        tx_type = DCT_DCT;
    else {
        if (plane_type == PLANE_TYPE_Y) {
            //const int32_t txk_type_idx =
            //    av1_get_txk_type_index(/*mbmi->*/sb_type, blk_row, blk_col);
            //tx_type = mbmi->txk_type[txk_type_idx];
        } else if (is_inter /*is_inter_block(mbmi)*/) {
            // scale back to y plane's coordinate
            //blk_row <<= pd->subsampling_y;
            //blk_col <<= pd->subsampling_x;
            //const int32_t txk_type_idx =
            //    av1_get_txk_type_index(mbmi->sb_type, blk_row, blk_col);
            //tx_type = mbmi->txk_type[txk_type_idx];
        } else {
            // In intra mode, uv planes don't share the same prediction mode as y
            // plane, so the tx_type should not be shared
            tx_type = intra_mode_to_tx_type(&mbmi.block_mi, PLANE_TYPE_UV);
        }
    }
    ASSERT(tx_type < TX_TYPES);
    if (!av1_ext_tx_used[tx_set_type][tx_type]) return DCT_DCT;
    return tx_type;
}

#if !REMOVE_UNUSED_CODE_PH2
void inject_intra_candidates_ois(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                                 SuperBlock *sb_ptr, uint32_t *candidate_total_cnt) {
    uint32_t               can_total_cnt   = 0;
    ModeDecisionCandidate *candidate_array = context_ptr->fast_candidate_array;
    EbBool                 disable_cfl_flag =
        (MAX(context_ptr->blk_geom->bheight, context_ptr->blk_geom->bwidth) > 32) ? EB_TRUE
                                                                                  : EB_FALSE;
#if ADDED_CFL_OFF
    disable_cfl_flag = context_ptr->md_disable_cfl ? EB_TRUE : disable_cfl_flag;
#endif

    SequenceControlSet *scs_ptr = (SequenceControlSet*)pcs_ptr->scs_wrapper_ptr->object_ptr;
    if (scs_ptr->static_config.disable_cfl_flag != DEFAULT && !disable_cfl_flag)
        // if disable_cfl_flag == 1 then it doesn't matter what cli says otherwise change it to cli
        disable_cfl_flag = (EbBool)scs_ptr->static_config.disable_cfl_flag;

    OisSbResults *ois_sb_results_ptr = pcs_ptr->parent_pcs_ptr->ois_sb_results[sb_ptr->index];
    OisCandidate *ois_blk_ptr =
        ois_sb_results_ptr
            ->ois_candidate_array[ep_to_pa_block_index[context_ptr->blk_geom->blkidx_mds]];
    uint8_t total_intra_luma_mode =
        ois_sb_results_ptr
            ->total_ois_intra_candidate[ep_to_pa_block_index[context_ptr->blk_geom->blkidx_mds]];
    FrameHeader *frm_hdr = &pcs_ptr->parent_pcs_ptr->frm_hdr;

    for (uint8_t intra_candidate_counter = 0; intra_candidate_counter < total_intra_luma_mode;
         ++intra_candidate_counter) {
        int intra_mode = ois_blk_ptr[can_total_cnt].intra_mode;
        assert(intra_mode < INTRA_MODES);
        if (av1_is_directional_mode((PredictionMode)intra_mode)) {
            int32_t angle_delta                 = ois_blk_ptr[can_total_cnt].angle_delta;
            candidate_array[can_total_cnt].type = INTRA_MODE;
            candidate_array[can_total_cnt].merge_flag = EB_FALSE;
#if MEM_OPT_PALETTE
            candidate_array[can_total_cnt].palette_info = NULL;
#else
            candidate_array[can_total_cnt].palette_info.pmi.palette_size[0] = 0;
            candidate_array[can_total_cnt].palette_info.pmi.palette_size[1] = 0;
#endif
            candidate_array[can_total_cnt].intra_luma_mode                  = intra_mode;
            candidate_array[can_total_cnt].distortion_ready                 = 1;
            candidate_array[can_total_cnt].me_distortion = ois_blk_ptr[can_total_cnt].distortion;
            candidate_array[can_total_cnt].use_intrabc   = 0;
            candidate_array[can_total_cnt].is_directional_mode_flag =
                (uint8_t)av1_is_directional_mode((PredictionMode)intra_mode);
            candidate_array[can_total_cnt].angle_delta[PLANE_TYPE_Y] = angle_delta;
            candidate_array[can_total_cnt].intra_chroma_mode =
                disable_cfl_flag
                    ? intra_luma_to_chroma[intra_mode]
                    : context_ptr->chroma_level <= CHROMA_MODE_1 ? UV_CFL_PRED : UV_DC_PRED;
            candidate_array[can_total_cnt].cfl_alpha_signs = 0;
            candidate_array[can_total_cnt].cfl_alpha_idx   = 0;
            candidate_array[can_total_cnt].is_directional_chroma_mode_flag =
                (uint8_t)av1_is_directional_mode(
                    (PredictionMode)candidate_array[can_total_cnt].intra_chroma_mode);
            candidate_array[can_total_cnt].angle_delta[PLANE_TYPE_UV] = 0;

            candidate_array[can_total_cnt].transform_type[0] = DCT_DCT;

            if (candidate_array[can_total_cnt].intra_chroma_mode == UV_CFL_PRED)
                candidate_array[can_total_cnt].transform_type_uv = DCT_DCT;
            else
                candidate_array[can_total_cnt].transform_type_uv = av1_get_tx_type(
                    context_ptr->blk_geom->bsize,
                    0,
                    (PredictionMode)candidate_array[can_total_cnt].intra_luma_mode,
                    (UvPredictionMode)candidate_array[can_total_cnt].intra_chroma_mode,
                    PLANE_TYPE_UV,
                    0,
                    0,
                    0,
                    context_ptr->blk_geom->txsize_uv[0][0],
                    frm_hdr->reduced_tx_set);
            candidate_array[can_total_cnt].ref_frame_type = INTRA_FRAME;
            candidate_array[can_total_cnt].pred_mode      = (PredictionMode)intra_mode;
            candidate_array[can_total_cnt].motion_mode    = SIMPLE_TRANSLATION;
            INCRMENT_CAND_TOTAL_COUNT(can_total_cnt);
        } else {
            candidate_array[can_total_cnt].type                             = INTRA_MODE;
            candidate_array[can_total_cnt].merge_flag                       = EB_FALSE;
#if MEM_OPT_PALETTE
            candidate_array[can_total_cnt].palette_info = NULL;
#else
            candidate_array[can_total_cnt].palette_info.pmi.palette_size[0] = 0;
            candidate_array[can_total_cnt].palette_info.pmi.palette_size[1] = 0;
#endif
            candidate_array[can_total_cnt].intra_luma_mode                  = intra_mode;
            candidate_array[can_total_cnt].distortion_ready                 = 1;
            candidate_array[can_total_cnt].me_distortion = ois_blk_ptr[can_total_cnt].distortion;
            candidate_array[can_total_cnt].use_intrabc   = 0;
            candidate_array[can_total_cnt].is_directional_mode_flag =
                (uint8_t)av1_is_directional_mode((PredictionMode)intra_mode);
            candidate_array[can_total_cnt].angle_delta[PLANE_TYPE_Y] = 0;
            candidate_array[can_total_cnt].intra_chroma_mode =
                disable_cfl_flag
                    ? intra_luma_to_chroma[intra_mode]
                    : context_ptr->chroma_level <= CHROMA_MODE_1 ? UV_CFL_PRED : UV_DC_PRED;

            candidate_array[can_total_cnt].cfl_alpha_signs = 0;
            candidate_array[can_total_cnt].cfl_alpha_idx   = 0;
            candidate_array[can_total_cnt].is_directional_chroma_mode_flag =
                (uint8_t)av1_is_directional_mode(
                    (PredictionMode)candidate_array[can_total_cnt].intra_chroma_mode);
            candidate_array[can_total_cnt].angle_delta[PLANE_TYPE_UV] = 0;
            candidate_array[can_total_cnt].transform_type[0]          = DCT_DCT;

            if (candidate_array[can_total_cnt].intra_chroma_mode == UV_CFL_PRED)
                candidate_array[can_total_cnt].transform_type_uv = DCT_DCT;
            else
                candidate_array[can_total_cnt].transform_type_uv = av1_get_tx_type(
                    context_ptr->blk_geom->bsize,
                    0,
                    (PredictionMode)candidate_array[can_total_cnt].intra_luma_mode,
                    (UvPredictionMode)candidate_array[can_total_cnt].intra_chroma_mode,
                    PLANE_TYPE_UV,
                    0,
                    0,
                    0,
                    context_ptr->blk_geom->txsize_uv[0][0],
                    frm_hdr->reduced_tx_set);
            candidate_array[can_total_cnt].ref_frame_type = INTRA_FRAME;
            candidate_array[can_total_cnt].pred_mode      = (PredictionMode)intra_mode;
            candidate_array[can_total_cnt].motion_mode    = SIMPLE_TRANSLATION;
            INCRMENT_CAND_TOTAL_COUNT(can_total_cnt);
        }
    }

    // update the total number of candidates injected
    (*candidate_total_cnt) = can_total_cnt;

    return;
}
#endif
double eb_av1_convert_qindex_to_q(int32_t qindex, AomBitDepth bit_depth);

// Values are now correlated to quantizer.
#if FP_MV_COST
static int sad_per_bit16lut_8[QINDEX_RANGE];
static int sad_per_bit_lut_10[QINDEX_RANGE];
static void init_me_luts_bd(int *bit16lut, int range,
    AomBitDepth bit_depth) {
    int i;
    // Initialize the sad lut tables using a formulaic calculation for now.
    // This is to make it easier to resolve the impact of experimental changes
    // to the quantizer tables.
    for (i = 0; i < range; i++) {
        const double q = eb_av1_convert_qindex_to_q(i, bit_depth);
        bit16lut[i] = (int)(0.0418 * q + 2.4107);
    }
}
void eb_av1_init_me_luts(void) {
    init_me_luts_bd(sad_per_bit16lut_8, QINDEX_RANGE, AOM_BITS_8);
    init_me_luts_bd(sad_per_bit_lut_10, QINDEX_RANGE, AOM_BITS_10);
}
#else
static int sad_per_bit16lut_8[QINDEX_RANGE];
static int sad_per_bit4lut_8[QINDEX_RANGE];

static void init_me_luts_bd(int *bit16lut, int *bit4lut, int range, AomBitDepth bit_depth) {
    int i;
    // Initialize the sad lut tables using a formulaic calculation for now.
    // This is to make it easier to resolve the impact of experimental changes
    // to the quantizer tables.
    for (i = 0; i < range; i++) {
        const double q = eb_av1_convert_qindex_to_q(i, bit_depth);
        bit16lut[i]    = (int)(0.0418 * q + 2.4107);
        bit4lut[i]     = (int)(0.063 * q + 2.742);
    }
}

void eb_av1_init_me_luts(void) {
    init_me_luts_bd(sad_per_bit16lut_8, sad_per_bit4lut_8, QINDEX_RANGE, AOM_BITS_8);
}
#endif
static INLINE int mv_check_bounds(const MvLimits *mv_limits, const MV *mv) {
    return (mv->row >> 3) < mv_limits->row_min || (mv->row >> 3) > mv_limits->row_max ||
           (mv->col >> 3) < mv_limits->col_min || (mv->col >> 3) > mv_limits->col_max;
}
void assert_release(int statement) {
    if (statement == 0) SVT_LOG("ASSERT_ERRRR\n");
}

void intra_bc_search(PictureControlSet *pcs, ModeDecisionContext *context_ptr,
                     const SequenceControlSet *scs, BlkStruct *blk_ptr, MV *dv_cand,
                     uint8_t *num_dv_cand) {
    IntraBcContext  x_st;
    IntraBcContext *x = &x_st;
    uint32_t full_lambda =  context_ptr->hbd_mode_decision ?
        context_ptr->full_lambda_md[EB_10_BIT_MD] :
        context_ptr->full_lambda_md[EB_8_BIT_MD];
    //fill x with what needed.
    x->is_exhaustive_allowed =
        context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4 ? 1 : 0;
    //CHKN crc calculator could be moved to mdContext and these init at init time.
    av1_crc_calculator_init(&x->crc_calculator1, 24, 0x5D6DCB);
    av1_crc_calculator_init(&x->crc_calculator2, 24, 0x864CFB);

    x->xd            = blk_ptr->av1xd;
    x->nmv_vec_cost  = context_ptr->md_rate_estimation_ptr->nmv_vec_cost;
    x->mv_cost_stack = context_ptr->md_rate_estimation_ptr->nmvcoststack;
    BlockSize bsize  = context_ptr->blk_geom->bsize;
    assert(bsize < BlockSizeS_ALL);
    FrameHeader *          frm_hdr    = &pcs->parent_pcs_ptr->frm_hdr;
    const Av1Common *const cm         = pcs->parent_pcs_ptr->av1_cm;
    MvReferenceFrame       ref_frame  = INTRA_FRAME;
    const int              num_planes = 3;
    MacroBlockD *          xd         = blk_ptr->av1xd;
    const TileInfo *       tile       = &xd->tile;
    const int              mi_row     = -xd->mb_to_top_edge / (8 * MI_SIZE);
    const int              mi_col     = -xd->mb_to_left_edge / (8 * MI_SIZE);
    const int              w          = block_size_wide[bsize];
    const int              h          = block_size_high[bsize];
    const int              sb_row     = mi_row >> scs->seq_header.sb_size_log2;
    const int              sb_col     = mi_col >> scs->seq_header.sb_size_log2;

    // Set up limit values for MV components.
    // Mv beyond the range do not produce new/different prediction block.
    const int mi_width   = mi_size_wide[bsize];
    const int mi_height  = mi_size_high[bsize];
    x->mv_limits.row_min = -(((mi_row + mi_height) * MI_SIZE) + AOM_INTERP_EXTEND);
    x->mv_limits.col_min = -(((mi_col + mi_width) * MI_SIZE) + AOM_INTERP_EXTEND);
    x->mv_limits.row_max = (cm->mi_rows - mi_row) * MI_SIZE + AOM_INTERP_EXTEND;
    x->mv_limits.col_max = (cm->mi_cols - mi_col) * MI_SIZE + AOM_INTERP_EXTEND;
    //set search paramters
    x->sadperbit16 = sad_per_bit16lut_8[frm_hdr->quantization_params.base_q_idx];
    x->errorperbit = full_lambda >> RD_EPB_SHIFT;
    x->errorperbit += (x->errorperbit == 0);
    //temp buffer for hash me
    for (int xi = 0; xi < 2; xi++)
        for (int yj = 0; yj < 2; yj++)
            x->hash_value_buffer[xi][yj] =
                (uint32_t *)malloc(AOM_BUFFER_SIZE_FOR_BLOCK_HASH * sizeof(uint32_t));

    IntMv nearestmv, nearmv;
    eb_av1_find_best_ref_mvs_from_stack(
        0,
#if MEM_OPT_MV_STACK
        context_ptr->ed_ref_mv_stack /*mbmi_ext*/,
#else
        context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
            .ed_ref_mv_stack /*mbmi_ext*/,
#endif
        xd,
        ref_frame,
        &nearestmv,
        &nearmv,
        0);
    if (nearestmv.as_int == INVALID_MV) nearestmv.as_int = 0;
    if (nearmv.as_int == INVALID_MV) nearmv.as_int = 0;
    IntMv dv_ref = nearestmv.as_int == 0 ? nearmv : nearestmv;
    if (dv_ref.as_int == 0)
        av1_find_ref_dv(&dv_ref, tile, scs->seq_header.sb_mi_size, mi_row, mi_col);
    // Ref DV should not have sub-pel.
    assert((dv_ref.as_mv.col & 7) == 0);
    assert((dv_ref.as_mv.row & 7) == 0);
#if MEM_OPT_MV_STACK
    context_ptr->ed_ref_mv_stack[INTRA_FRAME][0].this_mv = dv_ref;
#else
    context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
        .ed_ref_mv_stack[INTRA_FRAME][0]
        .this_mv = dv_ref;
#endif

    /* pointer to current frame */
    Yv12BufferConfig cur_buf;
    link_eb_to_aom_buffer_desc_8bit(pcs->parent_pcs_ptr->enhanced_picture_ptr, &cur_buf);
    struct Buf2D yv12_mb[MAX_MB_PLANE];
    eb_av1_setup_pred_block(bsize, yv12_mb, &cur_buf, mi_row, mi_col);
    for (int i = 0; i < num_planes; ++i) x->xdplane[i].pre[0] = yv12_mb[i]; //ref in ME
    //setup src for DV search same as ref
    x->plane[0].src = x->xdplane[0].pre[0];

    enum IntrabcMotionDirection { IBC_MOTION_ABOVE, IBC_MOTION_LEFT, IBC_MOTION_DIRECTIONS };

    //up to two dv candidates will be generated
    //IBC Modes:   0: OFF 1:Slow   2:Faster   3:Fastest
    enum IntrabcMotionDirection max_dir =
        pcs->parent_pcs_ptr->ibc_mode > 2 ? IBC_MOTION_LEFT : IBC_MOTION_DIRECTIONS;

    for (enum IntrabcMotionDirection dir = IBC_MOTION_ABOVE; dir < max_dir; ++dir) {
        const MvLimits tmp_mv_limits = x->mv_limits;

        switch (dir) {
        case IBC_MOTION_ABOVE:
            x->mv_limits.col_min = (tile->mi_col_start - mi_col) * MI_SIZE;
            x->mv_limits.col_max = (tile->mi_col_end - mi_col) * MI_SIZE - w;
            x->mv_limits.row_min = (tile->mi_row_start - mi_row) * MI_SIZE;
            x->mv_limits.row_max = (sb_row * scs->seq_header.sb_mi_size - mi_row) * MI_SIZE - h;
            break;
        case IBC_MOTION_LEFT:
            x->mv_limits.col_min = (tile->mi_col_start - mi_col) * MI_SIZE;
            x->mv_limits.col_max = (sb_col * scs->seq_header.sb_mi_size - mi_col) * MI_SIZE - w;
            // TODO: Minimize the overlap between above and
            // left areas.
            x->mv_limits.row_min = (tile->mi_row_start - mi_row) * MI_SIZE;
            int bottom_coded_mi_edge =
                AOMMIN((sb_row + 1) * scs->seq_header.sb_mi_size, tile->mi_row_end);
            x->mv_limits.row_max = (bottom_coded_mi_edge - mi_row) * MI_SIZE - h;
            break;
        default: assert(0);
        }
        assert_release(x->mv_limits.col_min >= tmp_mv_limits.col_min);
        assert_release(x->mv_limits.col_max <= tmp_mv_limits.col_max);
        assert_release(x->mv_limits.row_min >= tmp_mv_limits.row_min);
        assert_release(x->mv_limits.row_max <= tmp_mv_limits.row_max);

        eb_av1_set_mv_search_range(&x->mv_limits, &dv_ref.as_mv);

        if (x->mv_limits.col_max < x->mv_limits.col_min ||
            x->mv_limits.row_max < x->mv_limits.row_min) {
            x->mv_limits = tmp_mv_limits;
            continue;
        }

        int step_param = 0;
        MV  mvp_full   = dv_ref.as_mv;
        mvp_full.col >>= 3;
        mvp_full.row >>= 3;
        const int sadpb   = x->sadperbit16;
        x->best_mv.as_int = 0;

#define INT_VAR_MAX 2147483647 // maximum (signed) int value

        const int bestsme = eb_av1_full_pixel_search(pcs,
                                                     x,
                                                     bsize,
                                                     &mvp_full,
                                                     step_param,
                                                     1,
                                                     0,
                                                     sadpb,
                                                     NULL,
                                                     &dv_ref.as_mv,
                                                     INT_VAR_MAX,
                                                     1,
                                                     (MI_SIZE * mi_col),
                                                     (MI_SIZE * mi_row),
                                                     1);

        x->mv_limits = tmp_mv_limits;
        if (bestsme == INT_VAR_MAX) continue;
        mvp_full = x->best_mv.as_mv;

        const MV dv = {.row = mvp_full.row * 8, .col = mvp_full.col * 8};
        if (mv_check_bounds(&x->mv_limits, &dv)) continue;
        if (!av1_is_dv_valid(dv, xd, mi_row, mi_col, bsize, scs->seq_header.sb_size_log2)) continue;

        // DV should not have sub-pel.
        assert_release((dv.col & 7) == 0);
        assert_release((dv.row & 7) == 0);

        //store output
        dv_cand[*num_dv_cand] = dv;
        (*num_dv_cand)++;
    }

    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++) free(x->hash_value_buffer[i][j]);
}
#if UPGRADE_SUBPEL
void svt_init_mv_cost_params(MV_COST_PARAMS *mv_cost_params,
    ModeDecisionContext *context_ptr,
#if FP_MV_COST
    const MV *ref_mv, uint8_t base_q_idx, uint32_t rdmult, uint8_t hbd_mode_decision) {
#else
    const MV *ref_mv, uint8_t base_q_idx) {
#endif
    mv_cost_params->ref_mv = ref_mv;
    mv_cost_params->full_ref_mv = get_fullmv_from_mv(ref_mv);
    mv_cost_params->mv_cost_type = MV_COST_ENTROPY;
#if FP_MV_COST
    mv_cost_params->error_per_bit = AOMMAX(rdmult >> RD_EPB_SHIFT, 1);
    mv_cost_params->sad_per_bit = hbd_mode_decision ? sad_per_bit_lut_10[base_q_idx] : sad_per_bit16lut_8[base_q_idx];
#else
    mv_cost_params->error_per_bit = AOMMAX(context_ptr->full_lambda_md[EB_8_BIT_MD] >> RD_EPB_SHIFT, 1);
    mv_cost_params->sad_per_bit = sad_per_bit16lut_8[base_q_idx];
#endif
    mv_cost_params->mvjcost = context_ptr->md_rate_estimation_ptr->nmv_vec_cost;
    mv_cost_params->mvcost[0] = context_ptr->md_rate_estimation_ptr->nmvcoststack[0];
    mv_cost_params->mvcost[1] = context_ptr->md_rate_estimation_ptr->nmvcoststack[1];
}
#endif
void inject_intra_bc_candidates(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                                const SequenceControlSet *scs_ptr, BlkStruct *blk_ptr,
                                uint32_t *cand_cnt) {
    MV      dv_cand[2];
    uint8_t num_dv_cand = 0;

    //perform dv-pred + search up to 2 dv(s)
    intra_bc_search(pcs_ptr, context_ptr, scs_ptr, blk_ptr, dv_cand, &num_dv_cand);

    ModeDecisionCandidate *cand_array = context_ptr->fast_candidate_array;
    uint32_t               dv_i;

    for (dv_i = 0; dv_i < num_dv_cand; dv_i++) {
#if MEM_OPT_PALETTE
        cand_array[*cand_cnt].palette_info = NULL;
#else
        cand_array[*cand_cnt].palette_info.pmi.palette_size[0] = 0;
        cand_array[*cand_cnt].palette_info.pmi.palette_size[1] = 0;
#endif
        cand_array[*cand_cnt].type                             = INTRA_MODE;
        cand_array[*cand_cnt].intra_luma_mode                  = DC_PRED;
        cand_array[*cand_cnt].distortion_ready                 = 0;
        cand_array[*cand_cnt].use_intrabc                      = 1;
        cand_array[*cand_cnt].is_directional_mode_flag         = 0;
        cand_array[*cand_cnt].angle_delta[PLANE_TYPE_Y]        = 0;
        cand_array[*cand_cnt].intra_chroma_mode                = UV_DC_PRED;
        cand_array[*cand_cnt].cfl_alpha_signs                  = 0;
        cand_array[*cand_cnt].cfl_alpha_idx                    = 0;
        cand_array[*cand_cnt].is_directional_chroma_mode_flag  = 0;
        cand_array[*cand_cnt].angle_delta[PLANE_TYPE_UV]       = 0;
        cand_array[*cand_cnt].transform_type[0]                = DCT_DCT;
        cand_array[*cand_cnt].transform_type_uv                = DCT_DCT;
        cand_array[*cand_cnt].ref_frame_type                   = INTRA_FRAME;
        cand_array[*cand_cnt].pred_mode                        = DC_PRED;
        cand_array[*cand_cnt].motion_mode                      = SIMPLE_TRANSLATION;
        //inter ralated
        cand_array[*cand_cnt].is_compound             = 0;
        cand_array[*cand_cnt].is_interintra_used      = 0;
        cand_array[*cand_cnt].merge_flag              = EB_FALSE;
        cand_array[*cand_cnt].prediction_direction[0] = UNI_PRED_LIST_0;
        cand_array[*cand_cnt].is_new_mv               = 0;
        cand_array[*cand_cnt].motion_vector_xl0       = dv_cand[dv_i].col;
        cand_array[*cand_cnt].motion_vector_yl0       = dv_cand[dv_i].row;
        cand_array[*cand_cnt].motion_vector_pred_x[REF_LIST_0] =
#if MEM_OPT_MV_STACK
            context_ptr->ed_ref_mv_stack[INTRA_FRAME][0].this_mv.as_mv.col;
#else
            context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                .ed_ref_mv_stack[INTRA_FRAME][0]
                .this_mv.as_mv.col;
#endif
        cand_array[*cand_cnt].motion_vector_pred_y[REF_LIST_0] =
#if MEM_OPT_MV_STACK
            context_ptr->ed_ref_mv_stack[INTRA_FRAME][0].this_mv.as_mv.row;
#else
            context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                .ed_ref_mv_stack[INTRA_FRAME][0]
                .this_mv.as_mv.row;
#endif
        cand_array[*cand_cnt].drl_index         = 0;
        cand_array[*cand_cnt].ref_mv_index      = 0;
        cand_array[*cand_cnt].interp_filters    = av1_broadcast_interp_filter(BILINEAR);
        cand_array[*cand_cnt].filter_intra_mode = FILTER_INTRA_MODES;
        INCRMENT_CAND_TOTAL_COUNT((*cand_cnt));
    }
}
// Indices are sign, integer, and fractional part of the gradient value
static const uint8_t gradient_to_angle_bin[2][7][16] = {
    {
        {6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
        {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
    },
    {
        {6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4},
        {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3},
        {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3},
        {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3},
        {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3},
        {3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2},
        {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
    },
};

/* clang-format off */
static const uint8_t mode_to_angle_bin[INTRA_MODES] = {
  0, 2, 6, 0, 4, 3, 5, 7, 1, 0,
  0,
};
void av1_get_gradient_hist_c(const uint8_t *src, int src_stride, int rows,
    int cols, uint64_t *hist) {
    src += src_stride;
    for (int r = 1; r < rows; ++r) {
        for (int c = 1; c < cols; ++c) {
            int dx = src[c] - src[c - 1];
            int dy = src[c] - src[c - src_stride];
            int index;
            const int temp = dx * dx + dy * dy;
            if (dy == 0) {
                index = 2;
            }
            else {
                const int sn = (dx > 0) ^ (dy > 0);
                dx = abs(dx);
                dy = abs(dy);
                const int remd = (dx % dy) * 16 / dy;
                const int quot = dx / dy;
                index = gradient_to_angle_bin[sn][AOMMIN(quot, 6)][AOMMIN(remd, 15)];
            }
            hist[index] += temp;
        }
        src += src_stride;
    }
}
#if UV_SEARCH_MODE_INJCECTION
 void angle_estimation(
#else
static void angle_estimation(
#endif
    const uint8_t *src,
    int src_stride,
    int rows,
    int cols,
    //BLOCK_SIZE bsize,
    uint8_t *directional_mode_skip_mask)
{
    // Check if angle_delta is used
    //if (!av1_use_angle_delta(bsize, need access to context)) return;

    uint64_t hist[DIRECTIONAL_MODES] = { 0 };
    //if (is_hbd)
    //    get_highbd_gradient_hist(src, src_stride, rows, cols, hist);
    //else
    av1_get_gradient_hist(src, src_stride, rows, cols, hist);

    int i;
    uint64_t hist_sum = 0;
    for (i = 0; i < DIRECTIONAL_MODES; ++i) hist_sum += hist[i];
    for (i = 0; i < INTRA_MODES; ++i) {
        if (av1_is_directional_mode(i)) {
            const uint8_t angle_bin = mode_to_angle_bin[i];
            uint64_t score = 2 * hist[angle_bin];
            int weight = 2;
            if (angle_bin > 0) {
                score += hist[angle_bin - 1];
                ++weight;
            }
            if (angle_bin < DIRECTIONAL_MODES - 1) {
                score += hist[angle_bin + 1];
                ++weight;
            }
            const int thresh = 10;
            if (score * thresh < hist_sum * weight) directional_mode_skip_mask[i] = 1;
        }
    }
}
// END of Function Declarations
void  inject_intra_candidates(
    PictureControlSet            *pcs_ptr,
    ModeDecisionContext          *context_ptr,
    const SequenceControlSet     *scs_ptr,
    SuperBlock                   *sb_ptr,
    EbBool                        dc_cand_only_flag,
    uint32_t                     *candidate_total_cnt){
#if ADD_SKIP_INTRA_SIGNAL
    if (context_ptr->skip_intra)
        return;
#endif
    (void)scs_ptr;
    (void)sb_ptr;
    FrameHeader *frm_hdr = &pcs_ptr->parent_pcs_ptr->frm_hdr;
    uint8_t                     intra_mode_start = DC_PRED;
    uint8_t                     intra_mode_end = dc_cand_only_flag ? DC_PRED :
                                                 context_ptr->md_enable_paeth ? PAETH_PRED :
                                                 context_ptr->md_enable_smooth ? SMOOTH_H_PRED : D67_PRED;
    uint8_t                     open_loop_intra_candidate;
    uint32_t                    cand_total_cnt = 0;
    EbBool                      use_angle_delta = av1_use_angle_delta(context_ptr->blk_geom->bsize, context_ptr->md_intra_angle_delta);
    uint8_t                     angle_delta_candidate_count = use_angle_delta ? 7 : 1;
    ModeDecisionCandidate    *cand_array = context_ptr->fast_candidate_array;
    EbBool                      disable_cfl_flag = (MAX(context_ptr->blk_geom->bheight, context_ptr->blk_geom->bwidth) > 32) ? EB_TRUE : EB_FALSE;
#if ADDED_CFL_OFF
    disable_cfl_flag = context_ptr->md_disable_cfl ? EB_TRUE : disable_cfl_flag;
#endif
    uint8_t                     disable_z2_prediction;
    uint8_t                     disable_angle_prediction;
    uint8_t directional_mode_skip_mask[INTRA_MODES] = { 0 };

    if (scs_ptr->static_config.disable_cfl_flag != DEFAULT && !disable_cfl_flag)
        // if disable_cfl_flag == 1 then it doesn't matter what cli says otherwise change it to cli
        disable_cfl_flag = (EbBool)scs_ptr->static_config.disable_cfl_flag;

#if !REMOVE_EDGE_SKIP_ANGLE_INTRA
    if (context_ptr->edge_based_skip_angle_intra && use_angle_delta)
    {
        EbPictureBufferDesc   *src_pic = pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr;
        uint8_t               *src_buf = src_pic->buffer_y + (context_ptr->blk_origin_x + src_pic->origin_x) + (context_ptr->blk_origin_y + src_pic->origin_y) * src_pic->stride_y;
        const int rows = block_size_high[context_ptr->blk_geom->bsize];
        const int cols = block_size_wide[context_ptr->blk_geom->bsize];
        angle_estimation(src_buf, src_pic->stride_y, rows, cols, /*context_ptr->blk_geom->bsize,*/directional_mode_skip_mask);
    }
#endif

    uint8_t     angle_delta_shift = 1;
    if (context_ptr->disable_angle_z2_intra_flag) {
        disable_angle_prediction = 1;
        angle_delta_candidate_count = 1;
        angle_delta_shift = 1;
        disable_z2_prediction = 1;
    } else if (pcs_ptr->parent_pcs_ptr->intra_pred_mode == 4) {
        if (pcs_ptr->slice_type == I_SLICE) {
            intra_mode_end = context_ptr->md_enable_paeth ? PAETH_PRED :
                             context_ptr->md_enable_smooth ? SMOOTH_H_PRED : D67_PRED;
            angle_delta_candidate_count = use_angle_delta ? 5 : 1;
            disable_angle_prediction = 0;
            angle_delta_shift = 2;
            disable_z2_prediction = 0;
        }
        else {
            intra_mode_end = DC_PRED;
            disable_angle_prediction = 1;
            angle_delta_candidate_count = 1;
            angle_delta_shift = 1;
            disable_z2_prediction = 0;
        }
    } else if (pcs_ptr->parent_pcs_ptr->intra_pred_mode == 3){
        disable_z2_prediction       = 0;
        disable_angle_prediction    = 1;
    } else if (pcs_ptr->parent_pcs_ptr->intra_pred_mode == 2) {
        disable_z2_prediction       = 0;
        disable_angle_prediction    = (context_ptr->blk_geom->sq_size > 16 ||
                                       context_ptr->blk_geom->bwidth == 4 ||
                                       context_ptr->blk_geom->bheight == 4) ? 1 : 0;
    } else if (pcs_ptr->parent_pcs_ptr->intra_pred_mode == 1) {
        disable_z2_prediction       = (context_ptr->blk_geom->sq_size > 16 ||
                                       context_ptr->blk_geom->bwidth == 4 ||
                                       context_ptr->blk_geom->bheight == 4) ? 1 : 0;
        disable_angle_prediction    = 0;
        if (context_ptr->blk_geom->sq_size > 16 ||
            context_ptr->blk_geom->bwidth == 4 ||
            context_ptr->blk_geom->bheight == 4)
            angle_delta_candidate_count = 1;
    } else {
        disable_z2_prediction       = 0;
        disable_angle_prediction    = 0;
    }
#if MR_MODE
    disable_z2_prediction       = 0;
    disable_angle_prediction    = 0;
#endif
    for (open_loop_intra_candidate = intra_mode_start; open_loop_intra_candidate <= intra_mode_end ; ++open_loop_intra_candidate) {
        if (av1_is_directional_mode((PredictionMode)open_loop_intra_candidate)) {
            if (!disable_angle_prediction &&
                directional_mode_skip_mask[(PredictionMode)open_loop_intra_candidate] == 0) {
                for (uint8_t angle_delta_counter = 0; angle_delta_counter < angle_delta_candidate_count; ++angle_delta_counter) {
                    int32_t angle_delta = CLIP( angle_delta_shift * (angle_delta_candidate_count == 1 ? 0 : angle_delta_counter - (angle_delta_candidate_count >> 1)), -3 , 3);
                    int32_t  p_angle = mode_to_angle_map[(PredictionMode)open_loop_intra_candidate] + angle_delta * ANGLE_STEP;
                    if (!disable_z2_prediction || (p_angle <= 90 || p_angle >= 180)) {
                        cand_array[cand_total_cnt].type = INTRA_MODE;
                        cand_array[cand_total_cnt].merge_flag = EB_FALSE;
#if MEM_OPT_PALETTE
                        cand_array[cand_total_cnt].palette_info = NULL;
#else
                        cand_array[cand_total_cnt].palette_info.pmi.palette_size[0] = 0;
                        cand_array[cand_total_cnt].palette_info.pmi.palette_size[1] = 0;
#endif
                        cand_array[cand_total_cnt].intra_luma_mode = open_loop_intra_candidate;
                        cand_array[cand_total_cnt].distortion_ready = 0;
                        cand_array[cand_total_cnt].use_intrabc = 0;
                        cand_array[cand_total_cnt].filter_intra_mode = FILTER_INTRA_MODES;
                        cand_array[cand_total_cnt].is_directional_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)open_loop_intra_candidate);
                        cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_Y] = angle_delta;
                        // Search the best independent intra chroma mode
                        if (context_ptr->chroma_level == CHROMA_MODE_0) {
                            cand_array[cand_total_cnt].intra_chroma_mode = disable_cfl_flag ?
                                context_ptr->best_uv_mode[open_loop_intra_candidate][MAX_ANGLE_DELTA + cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_Y]] :
                                UV_CFL_PRED ;
                            cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_UV] = disable_cfl_flag ?
                                context_ptr->best_uv_angle[cand_array[cand_total_cnt].intra_luma_mode][MAX_ANGLE_DELTA + cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_Y]] : 0;
                            cand_array[cand_total_cnt].is_directional_chroma_mode_flag = disable_cfl_flag ?
                                (uint8_t)av1_is_directional_mode((PredictionMode)(context_ptr->best_uv_mode[cand_array[cand_total_cnt].intra_luma_mode][MAX_ANGLE_DELTA + cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_Y]])) : 0;
                        }
                        else {
                            // Hsan/Omar: why the restriction below ? (i.e. disable_ang_uv)
                            const int32_t disable_ang_uv = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) && context_ptr->blk_geom->has_uv ? 1 : 0;
                            cand_array[cand_total_cnt].intra_chroma_mode = disable_cfl_flag ?
                                intra_luma_to_chroma[open_loop_intra_candidate] :
                                (context_ptr->chroma_level == CHROMA_MODE_1) ?
                                UV_CFL_PRED :
                                UV_DC_PRED;
                            cand_array[cand_total_cnt].intra_chroma_mode = disable_ang_uv && av1_is_directional_mode(cand_array[cand_total_cnt].intra_chroma_mode) ?
                                UV_DC_PRED : cand_array[cand_total_cnt].intra_chroma_mode;
                            cand_array[cand_total_cnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)cand_array[cand_total_cnt].intra_chroma_mode);
                            cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_UV] = 0;
                        }
                        cand_array[cand_total_cnt].cfl_alpha_signs = 0;
                        cand_array[cand_total_cnt].cfl_alpha_idx = 0;
                        cand_array[cand_total_cnt].transform_type[0] = DCT_DCT;

                        if (cand_array[cand_total_cnt].intra_chroma_mode == UV_CFL_PRED)
                            cand_array[cand_total_cnt].transform_type_uv = DCT_DCT;
                        else
                            cand_array[cand_total_cnt].transform_type_uv =
                            av1_get_tx_type(
                                context_ptr->blk_geom->bsize,
                                0,
                                (PredictionMode)cand_array[cand_total_cnt].intra_luma_mode,
                                (UvPredictionMode)cand_array[cand_total_cnt].intra_chroma_mode,
                                PLANE_TYPE_UV,
                                0,
                                0,
                                0,
                                context_ptr->blk_geom->txsize_uv[0][0],
                                frm_hdr->reduced_tx_set);
                        cand_array[cand_total_cnt].ref_frame_type = INTRA_FRAME;
                        cand_array[cand_total_cnt].pred_mode = (PredictionMode)open_loop_intra_candidate;
                        cand_array[cand_total_cnt].motion_mode = SIMPLE_TRANSLATION;
                        cand_array[cand_total_cnt].is_interintra_used = 0;
                        INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
                    }
            }
        }
        }
        else {
            cand_array[cand_total_cnt].type = INTRA_MODE;
            cand_array[cand_total_cnt].merge_flag = EB_FALSE;
#if MEM_OPT_PALETTE
            cand_array[cand_total_cnt].palette_info = NULL;
#else
            cand_array[cand_total_cnt].palette_info.pmi.palette_size[0] = 0;
            cand_array[cand_total_cnt].palette_info.pmi.palette_size[1] = 0;
#endif
            cand_array[cand_total_cnt].intra_luma_mode = open_loop_intra_candidate;
            cand_array[cand_total_cnt].distortion_ready = 0;
            cand_array[cand_total_cnt].use_intrabc = 0;
            cand_array[cand_total_cnt].filter_intra_mode = FILTER_INTRA_MODES;
            cand_array[cand_total_cnt].is_directional_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)open_loop_intra_candidate);
            cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_Y] = 0;
            // Search the best independent intra chroma mode
            if (context_ptr->chroma_level == CHROMA_MODE_0) {
                cand_array[cand_total_cnt].intra_chroma_mode = disable_cfl_flag ?
                    context_ptr->best_uv_mode[open_loop_intra_candidate][MAX_ANGLE_DELTA + cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_Y]] :
                    UV_CFL_PRED;
                cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_UV] = disable_cfl_flag ?
                    context_ptr->best_uv_angle[cand_array[cand_total_cnt].intra_luma_mode][MAX_ANGLE_DELTA + cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_Y]] : 0;
                cand_array[cand_total_cnt].is_directional_chroma_mode_flag = disable_cfl_flag ?
                    (uint8_t)av1_is_directional_mode((PredictionMode)(context_ptr->best_uv_mode[cand_array[cand_total_cnt].intra_luma_mode][MAX_ANGLE_DELTA + cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_Y]])) : 0;
            }
            else {
                // Hsan/Omar: why the restriction below ? (i.e. disable_ang_uv)
                const int32_t disable_ang_uv = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) && context_ptr->blk_geom->has_uv ? 1 : 0;
                cand_array[cand_total_cnt].intra_chroma_mode = disable_cfl_flag ?
                    intra_luma_to_chroma[open_loop_intra_candidate] :
                    (context_ptr->chroma_level == CHROMA_MODE_1) ?
                        UV_CFL_PRED :
                        UV_DC_PRED;

                cand_array[cand_total_cnt].intra_chroma_mode = disable_ang_uv && av1_is_directional_mode(cand_array[cand_total_cnt].intra_chroma_mode) ?
                    UV_DC_PRED : cand_array[cand_total_cnt].intra_chroma_mode;

                cand_array[cand_total_cnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)cand_array[cand_total_cnt].intra_chroma_mode);
                cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_UV] = 0;

            }
            cand_array[cand_total_cnt].cfl_alpha_signs = 0;
            cand_array[cand_total_cnt].cfl_alpha_idx = 0;
            cand_array[cand_total_cnt].transform_type[0] = DCT_DCT;

            if (cand_array[cand_total_cnt].intra_chroma_mode == UV_CFL_PRED)
                cand_array[cand_total_cnt].transform_type_uv = DCT_DCT;
            else
                cand_array[cand_total_cnt].transform_type_uv =
                av1_get_tx_type(
                    context_ptr->blk_geom->bsize,
                    0,
                    (PredictionMode)cand_array[cand_total_cnt].intra_luma_mode,
                    (UvPredictionMode)cand_array[cand_total_cnt].intra_chroma_mode,
                    PLANE_TYPE_UV,
                    0,
                    0,
                    0,
                    context_ptr->blk_geom->txsize_uv[0][0],
                    frm_hdr->reduced_tx_set);
            cand_array[cand_total_cnt].ref_frame_type = INTRA_FRAME;
            cand_array[cand_total_cnt].pred_mode = (PredictionMode)open_loop_intra_candidate;
            cand_array[cand_total_cnt].motion_mode = SIMPLE_TRANSLATION;
            cand_array[cand_total_cnt].is_interintra_used = 0;
            INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
        }
    }

    // update the total number of candidates injected
    (*candidate_total_cnt) = cand_total_cnt;

    return;
}
// END of Function Declarations
void  inject_filter_intra_candidates(
    PictureControlSet            *pcs_ptr,
    ModeDecisionContext          *context_ptr,
    uint32_t                     *candidate_total_cnt){
#if ADD_SKIP_INTRA_SIGNAL
    if (context_ptr->skip_intra)
        return;
#endif
    FilterIntraMode             intra_mode_start = FILTER_DC_PRED;
    FilterIntraMode             intra_mode_end   = FILTER_INTRA_MODES;

    FilterIntraMode             filter_intra_mode;
    uint32_t                    cand_total_cnt = *candidate_total_cnt;
    ModeDecisionCandidate      *cand_array = context_ptr->fast_candidate_array;

    EbBool                      disable_cfl_flag = (MAX(context_ptr->blk_geom->bheight, context_ptr->blk_geom->bwidth) > 32) ? EB_TRUE : EB_FALSE;
#if ADDED_CFL_OFF
    disable_cfl_flag = context_ptr->md_disable_cfl ? EB_TRUE : disable_cfl_flag;
#endif
    SequenceControlSet *scs_ptr = (SequenceControlSet*)pcs_ptr->scs_wrapper_ptr->object_ptr;
    if (scs_ptr->static_config.disable_cfl_flag != DEFAULT && !disable_cfl_flag)
        // if disable_cfl_flag == 1 then it doesn't matter what cli says otherwise change it to cli
        disable_cfl_flag = (EbBool)scs_ptr->static_config.disable_cfl_flag;

    FrameHeader *frm_hdr = &pcs_ptr->parent_pcs_ptr->frm_hdr;

    for (filter_intra_mode = intra_mode_start; filter_intra_mode < intra_mode_end ; ++filter_intra_mode) {

            if (filter_intra_mode == FILTER_PAETH_PRED && !context_ptr->md_enable_paeth)
                continue;

            cand_array[cand_total_cnt].type = INTRA_MODE;
            cand_array[cand_total_cnt].merge_flag = EB_FALSE;
            cand_array[cand_total_cnt].intra_luma_mode = DC_PRED;
            cand_array[cand_total_cnt].distortion_ready = 0;
            cand_array[cand_total_cnt].use_intrabc = 0;
            cand_array[cand_total_cnt].filter_intra_mode = filter_intra_mode;
            cand_array[cand_total_cnt].is_directional_mode_flag = 0;
#if MEM_OPT_PALETTE
            cand_array[cand_total_cnt].palette_info = NULL;
#else
            cand_array[cand_total_cnt].palette_info.pmi.palette_size[0] = 0;
            cand_array[cand_total_cnt].palette_info.pmi.palette_size[1] = 0;
#endif
            cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_Y] = 0;

            // Search the best independent intra chroma mode
            if (context_ptr->chroma_level == CHROMA_MODE_0) {
                cand_array[cand_total_cnt].intra_chroma_mode  = disable_cfl_flag ? context_ptr->best_uv_mode[fimode_to_intramode[filter_intra_mode]][MAX_ANGLE_DELTA + cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_Y]] : UV_CFL_PRED;

                cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_UV] = disable_cfl_flag ? context_ptr->best_uv_angle[fimode_to_intramode[filter_intra_mode]][MAX_ANGLE_DELTA + cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_Y]] : 0;
                cand_array[cand_total_cnt].is_directional_chroma_mode_flag = disable_cfl_flag ? (uint8_t)av1_is_directional_mode((PredictionMode)(context_ptr->best_uv_mode[fimode_to_intramode[filter_intra_mode]][MAX_ANGLE_DELTA + cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_Y]])) : 0;

            }
            else {
                // Hsan/Omar: why the restriction below ? (i.e. disable_ang_uv)
                const int32_t disable_ang_uv = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) && context_ptr->blk_geom->has_uv ? 1 : 0;
                cand_array[cand_total_cnt].intra_chroma_mode = disable_cfl_flag ? intra_luma_to_chroma[fimode_to_intramode[filter_intra_mode]] :
                    (context_ptr->chroma_level == CHROMA_MODE_1) ?
                        UV_CFL_PRED :
                        UV_DC_PRED;

                cand_array[cand_total_cnt].intra_chroma_mode =  disable_ang_uv && av1_is_directional_mode(cand_array[cand_total_cnt].intra_chroma_mode) ?
                    UV_DC_PRED : cand_array[cand_total_cnt].intra_chroma_mode;

                cand_array[cand_total_cnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)cand_array[cand_total_cnt].intra_chroma_mode);
                cand_array[cand_total_cnt].angle_delta[PLANE_TYPE_UV] = 0;
            }

            cand_array[cand_total_cnt].cfl_alpha_signs = 0;
            cand_array[cand_total_cnt].cfl_alpha_idx = 0;
            cand_array[cand_total_cnt].transform_type[0] = DCT_DCT;

            if (cand_array[cand_total_cnt].intra_chroma_mode == UV_CFL_PRED)
                cand_array[cand_total_cnt].transform_type_uv = DCT_DCT;
            else
                cand_array[cand_total_cnt].transform_type_uv =
                av1_get_tx_type(
                    context_ptr->blk_geom->bsize,
                    0,
                    (PredictionMode)cand_array[cand_total_cnt].intra_luma_mode,
                    (UvPredictionMode)cand_array[cand_total_cnt].intra_chroma_mode,
                    PLANE_TYPE_UV,
                    0,
                    0,
                    0,
                    context_ptr->blk_geom->txsize_uv[0][0],
                    frm_hdr->reduced_tx_set);

            cand_array[cand_total_cnt].ref_frame_type = INTRA_FRAME;
            cand_array[cand_total_cnt].pred_mode = DC_PRED;
            cand_array[cand_total_cnt].motion_mode = SIMPLE_TRANSLATION;
            cand_array[cand_total_cnt].is_interintra_used = 0;
            INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);
    }

    // update the total number of candidates injected
    (*candidate_total_cnt) = cand_total_cnt;

    return;
}
#if INJECT_BACKUP_CANDIDATE
void inject_zz_backup_candidate(
    struct ModeDecisionContext *context_ptr,
    uint32_t *candidate_total_cnt) {
    ModeDecisionCandidate *cand_array = context_ptr->fast_candidate_array;
    IntMv                  best_pred_mv[2] = { {0}, {0} };
    uint32_t               cand_total_cnt = (*candidate_total_cnt);

    cand_array[cand_total_cnt].type = INTER_MODE;
    cand_array[cand_total_cnt].distortion_ready = 0;
    cand_array[cand_total_cnt].use_intrabc = 0;
    cand_array[cand_total_cnt].merge_flag = EB_FALSE;
    cand_array[cand_total_cnt].prediction_direction[0] = (EbPredDirection)0;
    cand_array[cand_total_cnt].inter_mode = NEWMV;
    cand_array[cand_total_cnt].pred_mode = NEWMV;
    cand_array[cand_total_cnt].motion_mode = SIMPLE_TRANSLATION;
    cand_array[cand_total_cnt].is_compound = 0;
    cand_array[cand_total_cnt].is_new_mv = 1;
    cand_array[cand_total_cnt].drl_index = 0;

    // zz
    cand_array[cand_total_cnt].motion_vector_xl0 = 0;
    cand_array[cand_total_cnt].motion_vector_yl0 = 0;

    // will be needed later by the rate estimation
    cand_array[cand_total_cnt].ref_mv_index = 0;
    cand_array[cand_total_cnt].ref_frame_type = svt_get_ref_frame_type(REF_LIST_0, 0);
    cand_array[cand_total_cnt].ref_frame_index_l0 = 0;
    cand_array[cand_total_cnt].ref_frame_index_l1 = -1;

    cand_array[cand_total_cnt].transform_type[0] = DCT_DCT;
    cand_array[cand_total_cnt].transform_type_uv = DCT_DCT;

    choose_best_av1_mv_pred(context_ptr,
        cand_array[cand_total_cnt].md_rate_estimation_ptr,
        context_ptr->blk_ptr,
        cand_array[cand_total_cnt].ref_frame_type,
        cand_array[cand_total_cnt].is_compound,
        cand_array[cand_total_cnt].pred_mode,
        cand_array[cand_total_cnt].motion_vector_xl0,
        cand_array[cand_total_cnt].motion_vector_yl0,
        0,
        0,
        &cand_array[cand_total_cnt].drl_index,
        best_pred_mv);

    cand_array[cand_total_cnt].motion_vector_pred_x[REF_LIST_0] = best_pred_mv[0].as_mv.col;
    cand_array[cand_total_cnt].motion_vector_pred_y[REF_LIST_0] = best_pred_mv[0].as_mv.row;

    cand_array[cand_total_cnt].is_interintra_used = 0;
    cand_array[cand_total_cnt].motion_mode = SIMPLE_TRANSLATION;

    INCRMENT_CAND_TOTAL_COUNT(cand_total_cnt);

    // update the total number of candidates injected
    (*candidate_total_cnt) = cand_total_cnt;
}
#endif
int svt_av1_allow_palette(int allow_palette,
    BlockSize sb_type) {
    assert(sb_type < BlockSizeS_ALL);
    return allow_palette && block_size_wide[sb_type] <= 64 &&
        block_size_high[sb_type] <= 64 && sb_type >= BLOCK_8X8;
}
void  search_palette_luma(
    PictureControlSet            *pcs_ptr,
    ModeDecisionContext          *context_ptr,
    PaletteInfo                 *palette_cand,
    uint32_t                     *tot_palette_cands);

void  inject_palette_candidates(
    PictureControlSet            *pcs_ptr,
    ModeDecisionContext          *context_ptr,
    uint32_t                       *candidate_total_cnt) {



    uint32_t                  can_total_cnt = *candidate_total_cnt;
    ModeDecisionCandidate    *cand_array = context_ptr->fast_candidate_array;
    EbBool                    disable_cfl_flag = (MAX(context_ptr->blk_geom->bheight, context_ptr->blk_geom->bwidth) > 32) ? EB_TRUE : EB_FALSE;
 #if ADDED_CFL_OFF
    disable_cfl_flag = context_ptr->md_disable_cfl ? EB_TRUE : disable_cfl_flag;
#endif
    uint32_t cand_i;
    uint32_t tot_palette_cands = 0;
    PaletteInfo    *palette_cand_array = context_ptr->palette_cand_array;

    SequenceControlSet *scs_ptr = (SequenceControlSet*)pcs_ptr->scs_wrapper_ptr->object_ptr;
    if (scs_ptr->static_config.disable_cfl_flag != DEFAULT && !disable_cfl_flag)
        // if disable_cfl_flag == 1 then it doesn't matter what cli says otherwise change it to cli
        disable_cfl_flag = (EbBool)scs_ptr->static_config.disable_cfl_flag;

    search_palette_luma(
        pcs_ptr,
        context_ptr,
        palette_cand_array,
        &tot_palette_cands);

    for (cand_i = 0; cand_i < tot_palette_cands; ++cand_i) {
        cand_array[can_total_cnt].is_interintra_used = 0;
        palette_cand_array[cand_i].pmi.palette_size[1] = 0;
#if MEM_OPT_PALETTE
        cand_array[can_total_cnt].palette_info = &palette_cand_array[cand_i];
#else
        eb_memcpy(cand_array[can_total_cnt].palette_info.color_idx_map, palette_cand_array[cand_i].color_idx_map, 64 * 64);
        eb_memcpy(&cand_array[can_total_cnt].palette_info.pmi, &palette_cand_array[cand_i].pmi, sizeof(PaletteModeInfo));
#endif
        assert(palette_cand_array[cand_i].pmi.palette_size[0] < 9);
        //to re check these fields
        cand_array[can_total_cnt].type = INTRA_MODE;
        cand_array[can_total_cnt].merge_flag = EB_FALSE;
        cand_array[can_total_cnt].intra_luma_mode = DC_PRED;
        cand_array[can_total_cnt].distortion_ready = 0;
        cand_array[can_total_cnt].use_intrabc = 0;

        cand_array[can_total_cnt].filter_intra_mode = FILTER_INTRA_MODES;

        cand_array[can_total_cnt].is_directional_mode_flag = 0;

        cand_array[can_total_cnt].angle_delta[PLANE_TYPE_Y] = 0;
#if FIX_CHROMA_PALETTE_INTERACTION
        const int32_t disable_ang_uv = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) && context_ptr->blk_geom->has_uv ? 1 : 0;
        cand_array[can_total_cnt].intra_chroma_mode = disable_cfl_flag ?
            intra_luma_to_chroma[DC_PRED] :
            (context_ptr->chroma_level <= CHROMA_MODE_1) ?
            UV_CFL_PRED :
            UV_DC_PRED;

        cand_array[can_total_cnt].intra_chroma_mode = disable_ang_uv && av1_is_directional_mode(cand_array[can_total_cnt].intra_chroma_mode) ?
            UV_DC_PRED : cand_array[can_total_cnt].intra_chroma_mode;

        cand_array[can_total_cnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)cand_array[can_total_cnt].intra_chroma_mode);
        cand_array[can_total_cnt].angle_delta[PLANE_TYPE_UV] = 0;
#else
        // Search the best independent intra chroma mode
        if (context_ptr->chroma_level == CHROMA_MODE_0) {
            cand_array[can_total_cnt].intra_chroma_mode = disable_cfl_flag ?
                context_ptr->best_uv_mode[DC_PRED][MAX_ANGLE_DELTA + cand_array[can_total_cnt].angle_delta[PLANE_TYPE_Y]] :
                UV_CFL_PRED;

            cand_array[can_total_cnt].angle_delta[PLANE_TYPE_UV] = disable_cfl_flag ?
                context_ptr->best_uv_angle[cand_array[can_total_cnt].intra_luma_mode][MAX_ANGLE_DELTA + cand_array[can_total_cnt].angle_delta[PLANE_TYPE_Y]] : 0;
            cand_array[can_total_cnt].is_directional_chroma_mode_flag = disable_cfl_flag ?
                (uint8_t)av1_is_directional_mode((PredictionMode)(context_ptr->best_uv_mode[cand_array[can_total_cnt].intra_luma_mode][MAX_ANGLE_DELTA + cand_array[can_total_cnt].angle_delta[PLANE_TYPE_Y]])) : 0;

        }
        else {
            // Hsan/Omar: why the restriction below ? (i.e. disable_ang_uv)
            const int32_t disable_ang_uv = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) && context_ptr->blk_geom->has_uv ? 1 : 0;
            cand_array[can_total_cnt].intra_chroma_mode = disable_cfl_flag ?
                intra_luma_to_chroma[DC_PRED] :
                (context_ptr->chroma_level == CHROMA_MODE_1) ?
                UV_CFL_PRED :
                UV_DC_PRED;

            cand_array[can_total_cnt].intra_chroma_mode = disable_ang_uv && av1_is_directional_mode(cand_array[can_total_cnt].intra_chroma_mode) ?
                UV_DC_PRED : cand_array[can_total_cnt].intra_chroma_mode;

            cand_array[can_total_cnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)cand_array[can_total_cnt].intra_chroma_mode);
            cand_array[can_total_cnt].angle_delta[PLANE_TYPE_UV] = 0;

        }
#endif
        cand_array[can_total_cnt].cfl_alpha_signs = 0;
        cand_array[can_total_cnt].cfl_alpha_idx = 0;
        cand_array[can_total_cnt].transform_type[0] = DCT_DCT;

        if (cand_array[can_total_cnt].intra_chroma_mode == UV_CFL_PRED)
            cand_array[can_total_cnt].transform_type_uv = DCT_DCT;
        else
            cand_array[can_total_cnt].transform_type_uv =

            av1_get_tx_type(
                context_ptr->blk_geom->bsize,
                0,
                (PredictionMode)cand_array[can_total_cnt].intra_luma_mode,
                (UvPredictionMode)cand_array[can_total_cnt].intra_chroma_mode,
                PLANE_TYPE_UV,
                0,
                0,
                0,
                context_ptr->blk_geom->txsize_uv[0][0],
                pcs_ptr->parent_pcs_ptr->frm_hdr.reduced_tx_set);

        cand_array[can_total_cnt].ref_frame_type = INTRA_FRAME;
        cand_array[can_total_cnt].pred_mode = (PredictionMode)DC_PRED;
        cand_array[can_total_cnt].motion_mode = SIMPLE_TRANSLATION;
        INCRMENT_CAND_TOTAL_COUNT(can_total_cnt);
    }

    // update the total number of candidates injected
    (*candidate_total_cnt) = can_total_cnt;

    return;
}
EbErrorType generate_md_stage_0_cand(
    SuperBlock          *sb_ptr,
    ModeDecisionContext *context_ptr,
    uint32_t            *candidate_total_count_ptr,
    PictureControlSet   *pcs_ptr)
{

#if !SHUT_PALETTE_BC_PD_PASS_0_1
    FrameHeader *frm_hdr = &pcs_ptr->parent_pcs_ptr->frm_hdr;
#endif
    const SequenceControlSet *scs_ptr = (SequenceControlSet*)pcs_ptr->scs_wrapper_ptr->object_ptr;
    const EB_SLICE slice_type = pcs_ptr->slice_type;
    uint32_t cand_total_cnt = 0;
    // Reset duplicates variables
    context_ptr->injected_mv_count_l0 = 0;
    context_ptr->injected_mv_count_l1 = 0;
    context_ptr->injected_mv_count_bipred = 0;
#if !SWITCH_MODE_BASED_ON_SQ_COEFF
    uint8_t sq_index = eb_log2f(context_ptr->blk_geom->sq_size) - 2;
    EbBool coeff_based_nsq_cand_reduction = EB_FALSE;
    if (slice_type != I_SLICE) {
        if (context_ptr->coeff_based_nsq_cand_reduction) {
            if (context_ptr->md_local_blk_unit[context_ptr->blk_geom->sqi_mds].avail_blk_flag)
                coeff_based_nsq_cand_reduction = context_ptr->blk_geom->shape == PART_N || context_ptr->parent_sq_has_coeff[sq_index] != 0 ? EB_FALSE : EB_TRUE;
        }
}
#endif
    //----------------------
    // Intra
    if (context_ptr->blk_geom->sq_size < 128) {
#if !REMOVE_UNUSED_CODE_PH2
        if (!context_ptr->dc_cand_only_flag && !coeff_based_nsq_cand_reduction && pcs_ptr->parent_pcs_ptr->intra_pred_mode >= 5 && context_ptr->blk_geom->sq_size > 4 && context_ptr->blk_geom->shape == PART_N)
            inject_intra_candidates_ois(
                pcs_ptr,
                context_ptr,
                sb_ptr,
                &cand_total_cnt);
        else
#endif
                inject_intra_candidates(
                    pcs_ptr,
                    context_ptr,
                    scs_ptr,
                    sb_ptr,
#if SWITCH_MODE_BASED_ON_SQ_COEFF
                    context_ptr->dc_cand_only_flag,
#else
                    context_ptr->dc_cand_only_flag || coeff_based_nsq_cand_reduction,
#endif
                &cand_total_cnt);
    }
#if !SWITCH_MODE_BASED_ON_SQ_COEFF
    if (!coeff_based_nsq_cand_reduction)
#endif
#if FILTER_INTRA_CLI
       if (context_ptr->md_filter_intra_level > 0 && av1_filter_intra_allowed_bsize(scs_ptr->seq_header.filter_intra_level, context_ptr->blk_geom->bsize))
#else
       if (context_ptr->md_filter_intra_mode > 0 && av1_filter_intra_allowed_bsize(scs_ptr->seq_header.enable_filter_intra, context_ptr->blk_geom->bsize))
#endif
            inject_filter_intra_candidates(
                pcs_ptr,
                context_ptr,
                &cand_total_cnt);

#if SHUT_PALETTE_BC_PD_PASS_0_1
    if (context_ptr->md_allow_intrabc)
#else
    if (frm_hdr->allow_intrabc)
#endif
        inject_intra_bc_candidates(
            pcs_ptr,
            context_ptr,
            scs_ptr,
            context_ptr->blk_ptr,
            &cand_total_cnt
        );
#if SHUT_PALETTE_BC_PD_PASS_0_1
    if (context_ptr->md_palette_level) {
#endif
    //can be removed later if need be
    for (uint16_t i = 0; i < cand_total_cnt; i++) {
#if MEM_OPT_PALETTE
        assert(context_ptr->fast_candidate_array[i].palette_info == NULL);
#else
        assert(context_ptr->fast_candidate_array[i].palette_info.pmi.palette_size[0] == 0);
        assert(context_ptr->fast_candidate_array[i].palette_info.pmi.palette_size[1] == 0);
#endif
    }
#if SHUT_PALETTE_BC_PD_PASS_0_1
    if (svt_av1_allow_palette(context_ptr->md_palette_level, context_ptr->blk_geom->bsize)) {
#else
    if (svt_av1_allow_palette(pcs_ptr->parent_pcs_ptr->palette_mode, context_ptr->blk_geom->bsize)) {
#endif
        inject_palette_candidates(
            pcs_ptr,
            context_ptr,
            &cand_total_cnt);
    }
    for (uint16_t i = 0; i < cand_total_cnt; i++) {
#if MEM_OPT_PALETTE
        assert(context_ptr->fast_candidate_array[i].palette_info == NULL ||
                context_ptr->fast_candidate_array[i].palette_info->pmi.palette_size[0] < 9);
        assert(context_ptr->fast_candidate_array[i].palette_info == NULL ||
                context_ptr->fast_candidate_array[i].palette_info->pmi.palette_size[1] == 0);
#else
        assert(context_ptr->fast_candidate_array[i].palette_info.pmi.palette_size[0] < 9);
        assert(context_ptr->fast_candidate_array[i].palette_info.pmi.palette_size[1] == 0);
#endif
    }
#if SHUT_PALETTE_BC_PD_PASS_0_1
    }
#endif
    if (slice_type != I_SLICE && context_ptr->inject_inter_candidates) {
            inject_inter_candidates(
                pcs_ptr,
                context_ptr,
                scs_ptr,
                sb_ptr,
#if !SWITCH_MODE_BASED_ON_SQ_COEFF
                coeff_based_nsq_cand_reduction,
#endif
                &cand_total_cnt);
    }
#if INJECT_BACKUP_CANDIDATE
    // For I_SLICE, DC is always injected, and therefore there is no a risk of no candidates @ md_syage_0()
    // For non I_SLICE, there is a risk of no candidates @ md_stage_0() because of the INTER candidates pruning techniques
    if (slice_type != I_SLICE && cand_total_cnt == 0) {
        inject_zz_backup_candidate(
            context_ptr,
            &cand_total_cnt);
    }
#endif
    *candidate_total_count_ptr = cand_total_cnt;
    CandClass  cand_class_it;
    memset(context_ptr->md_stage_0_count, 0, CAND_CLASS_TOTAL * sizeof(uint32_t));

    uint32_t cand_i;
    for (cand_i = 0; cand_i < cand_total_cnt; cand_i++)
    {
        ModeDecisionCandidate * cand_ptr = &context_ptr->fast_candidate_array[cand_i];

        if (cand_ptr->type == INTRA_MODE) {
            // Intra prediction
#if !CLASS_MERGING
                if (cand_ptr->filter_intra_mode == FILTER_INTRA_MODES) {
#endif
#if MEM_OPT_PALETTE
                  if (cand_ptr->palette_info == NULL ||
                          cand_ptr->palette_info->pmi.palette_size[0] == 0) {
#else
                  if (cand_ptr->palette_info.pmi.palette_size[0] == 0) {
#endif
                    cand_ptr->cand_class = CAND_CLASS_0;
                    context_ptr->md_stage_0_count[CAND_CLASS_0]++;
                  }
                  else {
#if CLASS_MERGING
                      // Palette Prediction
                     cand_ptr->cand_class = CAND_CLASS_3;
                     context_ptr->md_stage_0_count[CAND_CLASS_3]++;
#else
                     cand_ptr->cand_class = CAND_CLASS_7;
                     context_ptr->md_stage_0_count[CAND_CLASS_7]++;
#endif
                  }
#if !CLASS_MERGING
                }
                else {
                    cand_ptr->cand_class = CAND_CLASS_6;
                    context_ptr->md_stage_0_count[CAND_CLASS_6]++;
                }
#endif
        }
#if CLASS_MERGING

        else  if (cand_ptr->is_new_mv) {
            // MV Prediction
            cand_ptr->cand_class = CAND_CLASS_1;
            context_ptr->md_stage_0_count[CAND_CLASS_1]++;
        }
        else {
            //MVP Prediction
            cand_ptr->cand_class = CAND_CLASS_2;
            context_ptr->md_stage_0_count[CAND_CLASS_2]++;
        }
    }

#else
        else if (cand_ptr->inter_mode == GLOBALMV || cand_ptr->inter_mode == GLOBAL_GLOBALMV) {
            cand_ptr->cand_class = CAND_CLASS_8;
            context_ptr->md_stage_0_count[CAND_CLASS_8]++;
        }
        else {
            // Inter pred
            if (cand_ptr->motion_mode == OBMC_CAUSAL) {
                // OBMC
                cand_ptr->cand_class = CAND_CLASS_5;
                context_ptr->md_stage_0_count[CAND_CLASS_5]++;
            }
            else if (cand_ptr->is_compound == 0 ||
                    (cand_ptr->is_compound == 1 && cand_ptr->interinter_comp.type == COMPOUND_AVERAGE)) {
                if (cand_ptr->is_interintra_used && cand_ptr->is_compound == 0) {
                    // InterIntra
                    cand_ptr->cand_class = CAND_CLASS_4;
                    context_ptr->md_stage_0_count[CAND_CLASS_4]++;

                }
#if !REMOVE_COMBINE_CLASS12
                else
                if (context_ptr->combine_class12) {
                    cand_ptr->cand_class = CAND_CLASS_1;
                    context_ptr->md_stage_0_count[CAND_CLASS_1]++;
                }
#endif
                else {
                    if (cand_ptr->is_new_mv) {
                        // ME pred
                        cand_ptr->cand_class = CAND_CLASS_1;
                        context_ptr->md_stage_0_count[CAND_CLASS_1]++;
                    }
                    else {
                        // MV pred
                        cand_ptr->cand_class = CAND_CLASS_2;
                        context_ptr->md_stage_0_count[CAND_CLASS_2]++;
                    }
                }
            }
            else {
                // InterInter
                cand_ptr->cand_class = CAND_CLASS_3;
                context_ptr->md_stage_0_count[CAND_CLASS_3]++;
            }
        }
    }
#endif
    uint32_t fast_accum = 0;
    for (cand_class_it = CAND_CLASS_0; cand_class_it < CAND_CLASS_TOTAL; cand_class_it++) {
        fast_accum += context_ptr->md_stage_0_count[cand_class_it];
    }
    assert(fast_accum == cand_total_cnt);

#if LOG_MV_VALIDITY

    //check if final MV is within AV1 limits
    for (cand_i = 0; cand_i < cand_total_cnt; cand_i++)
    {
        ModeDecisionCandidate * cand_ptr = &context_ptr->fast_candidate_array[cand_i];

        if (cand_ptr->type == INTER_MODE) {
            if (cand_ptr->prediction_direction[0] == UNI_PRED_LIST_0 ||
                cand_ptr->prediction_direction[0] == BI_PRED)
                check_mv_validity(cand_ptr->motion_vector_xl0,
                    cand_ptr->motion_vector_yl0, 0);

            if (cand_ptr->prediction_direction[0] == UNI_PRED_LIST_1 ||
                cand_ptr->prediction_direction[0] == BI_PRED)
                check_mv_validity(cand_ptr->motion_vector_xl1,
                    cand_ptr->motion_vector_yl1, 0);

        }
    }
#endif
    return EB_ErrorNone;
}

/***************************************
* Full Mode Decision
***************************************/
uint32_t product_full_mode_decision(
    struct ModeDecisionContext   *context_ptr,
    BlkStruct                   *blk_ptr,
    ModeDecisionCandidateBuffer **buffer_ptr_array,
    uint32_t                      candidate_total_count,
    uint32_t                     *best_candidate_index_array,
#if !REMOVE_REF_FOR_RECT_PART
    uint8_t                       prune_ref_frame_for_rec_partitions,
#endif
    uint32_t                     *best_intra_mode)
{
#if !REMOVE_REF_FOR_RECT_PART
    uint32_t                  cand_index;
#endif
    uint64_t                  lowest_cost = 0xFFFFFFFFFFFFFFFFull;
    uint64_t                  lowest_intra_cost = 0xFFFFFFFFFFFFFFFFull;
    uint32_t                  lowest_cost_index = 0;
#if !REMOVE_REF_FOR_RECT_PART
    if (prune_ref_frame_for_rec_partitions) {
        if (context_ptr->blk_geom->shape == PART_N) {
            for (uint32_t i = 0; i < candidate_total_count; ++i) {
                cand_index = best_candidate_index_array[i];
                ModeDecisionCandidate *candidate_ptr = buffer_ptr_array[cand_index]->candidate_ptr;
                EbBool is_inter = (candidate_ptr->pred_mode >= NEARESTMV) ? EB_TRUE : EB_FALSE;
                EbBool is_simple_translation = (candidate_ptr->motion_mode != WARPED_CAUSAL) ? EB_TRUE : EB_FALSE;
                if (is_inter && is_simple_translation) {
                    uint8_t ref_frame_type = candidate_ptr->ref_frame_type;
                    assert(ref_frame_type < MAX_REF_TYPE_CAND);
                    context_ptr->ref_best_cost_sq_table[ref_frame_type] = *(buffer_ptr_array[cand_index]->full_cost_ptr);
                }

            }
            //Sort ref_frame by sq cost.
            uint32_t i, j, index;
            for (i = 0; i < MAX_REF_TYPE_CAND; ++i) {
                context_ptr->ref_best_ref_sq_table[i] = i;
            }
            for (i = 0; i < MAX_REF_TYPE_CAND - 1; ++i) {
                for (j = i + 1; j < MAX_REF_TYPE_CAND; ++j) {
                    if (context_ptr->ref_best_cost_sq_table[context_ptr->ref_best_ref_sq_table[j]] < context_ptr->ref_best_cost_sq_table[context_ptr->ref_best_ref_sq_table[i]]) {
                        index = context_ptr->ref_best_ref_sq_table[i];
                        context_ptr->ref_best_ref_sq_table[i] = context_ptr->ref_best_ref_sq_table[j];
                        context_ptr->ref_best_ref_sq_table[j] = index;
                    }
                }
            }
        }
    }
#endif

    ModeDecisionCandidate       *candidate_ptr;

    lowest_cost_index = best_candidate_index_array[0];

    // Find the candidate with the lowest cost
    for (uint32_t i = 0; i < candidate_total_count; ++i) {
#if REMOVE_REF_FOR_RECT_PART
        uint32_t cand_index = best_candidate_index_array[i];
#else
        cand_index = best_candidate_index_array[i];
#endif

        // Compute fullCostBis
        if ((*(buffer_ptr_array[cand_index]->full_cost_ptr) < lowest_intra_cost) && buffer_ptr_array[cand_index]->candidate_ptr->type == INTRA_MODE) {
            *best_intra_mode = buffer_ptr_array[cand_index]->candidate_ptr->pred_mode;
            lowest_intra_cost = *(buffer_ptr_array[cand_index]->full_cost_ptr);
        }

        if (*(buffer_ptr_array[cand_index]->full_cost_ptr) < lowest_cost) {
            lowest_cost_index = cand_index;
            lowest_cost = *(buffer_ptr_array[cand_index]->full_cost_ptr);
        }
    }

    candidate_ptr = buffer_ptr_array[lowest_cost_index]->candidate_ptr;
#if TPL_LAMBDA_IMP
    if (context_ptr->blk_lambda_tuning){
        // When lambda tuning is on, lambda of each block is set separately, however at interdepth decision the sb lambda is used
        uint32_t full_lambda = context_ptr->hbd_mode_decision ?
            context_ptr->full_sb_lambda_md[EB_10_BIT_MD] :
            context_ptr->full_sb_lambda_md[EB_8_BIT_MD];
        context_ptr->md_local_blk_unit[blk_ptr->mds_idx].cost =
            RDCOST(full_lambda,
                buffer_ptr_array[lowest_cost_index]->candidate_ptr->total_rate,
                ((uint64_t)buffer_ptr_array[lowest_cost_index]->candidate_ptr->full_distortion));
        context_ptr->md_local_blk_unit[blk_ptr->mds_idx].default_cost = context_ptr->md_local_blk_unit[blk_ptr->mds_idx].cost;
    }
    else {
        context_ptr->md_local_blk_unit[blk_ptr->mds_idx].cost = *(buffer_ptr_array[lowest_cost_index]->full_cost_ptr);
        context_ptr->md_local_blk_unit[blk_ptr->mds_idx].default_cost = *(buffer_ptr_array[lowest_cost_index]->full_cost_ptr);
        context_ptr->md_local_blk_unit[blk_ptr->mds_idx].cost = (context_ptr->md_local_blk_unit[blk_ptr->mds_idx].cost - buffer_ptr_array[lowest_cost_index]->candidate_ptr->chroma_distortion) + buffer_ptr_array[lowest_cost_index]->candidate_ptr->chroma_distortion_inter_depth;
    }
#else
    context_ptr->md_local_blk_unit[blk_ptr->mds_idx].cost = *(buffer_ptr_array[lowest_cost_index]->full_cost_ptr);
    context_ptr->md_local_blk_unit[blk_ptr->mds_idx].default_cost = *(buffer_ptr_array[lowest_cost_index]->full_cost_ptr);
    context_ptr->md_local_blk_unit[blk_ptr->mds_idx].cost = (context_ptr->md_local_blk_unit[blk_ptr->mds_idx].cost - buffer_ptr_array[lowest_cost_index]->candidate_ptr->chroma_distortion) + buffer_ptr_array[lowest_cost_index]->candidate_ptr->chroma_distortion_inter_depth;
#endif
    context_ptr->md_ep_pipe_sb[blk_ptr->mds_idx].merge_cost = *buffer_ptr_array[lowest_cost_index]->full_cost_merge_ptr;
    context_ptr->md_ep_pipe_sb[blk_ptr->mds_idx].skip_cost = *buffer_ptr_array[lowest_cost_index]->full_cost_skip_ptr;

    if (candidate_ptr->type == INTER_MODE && candidate_ptr->merge_flag == EB_TRUE)
        context_ptr->md_ep_pipe_sb[blk_ptr->mds_idx].chroma_distortion = buffer_ptr_array[lowest_cost_index]->candidate_ptr->chroma_distortion;
#if TPL_LAMBDA_IMP
    context_ptr->md_local_blk_unit[blk_ptr->mds_idx].full_distortion = (uint32_t)buffer_ptr_array[lowest_cost_index]->candidate_ptr->full_distortion;
#else
    context_ptr->md_local_blk_unit[blk_ptr->mds_idx].full_distortion = buffer_ptr_array[lowest_cost_index]->candidate_ptr->full_distortion;
#endif
    context_ptr->md_local_blk_unit[blk_ptr->mds_idx].chroma_distortion = (uint32_t)buffer_ptr_array[lowest_cost_index]->candidate_ptr->chroma_distortion;
    context_ptr->md_local_blk_unit[blk_ptr->mds_idx].chroma_distortion_inter_depth = (uint32_t)buffer_ptr_array[lowest_cost_index]->candidate_ptr->chroma_distortion_inter_depth;

    blk_ptr->prediction_mode_flag = candidate_ptr->type;
    blk_ptr->tx_depth = candidate_ptr->tx_depth;
    blk_ptr->skip_flag = candidate_ptr->skip_flag; // note, the skip flag is re-checked in the ENCDEC process
    blk_ptr->block_has_coeff = ((candidate_ptr->block_has_coeff) > 0) ? EB_TRUE : EB_FALSE;
    blk_ptr->quantized_dc[1][0] = buffer_ptr_array[lowest_cost_index]->candidate_ptr->quantized_dc[1][0];
    blk_ptr->quantized_dc[2][0] = buffer_ptr_array[lowest_cost_index]->candidate_ptr->quantized_dc[2][0];
    context_ptr->md_local_blk_unit[blk_ptr->mds_idx].count_non_zero_coeffs = candidate_ptr->count_non_zero_coeffs;
#if SB_MEM_OPT
    blk_ptr->use_intrabc = candidate_ptr->use_intrabc;
#else
    blk_ptr->av1xd->use_intrabc = candidate_ptr->use_intrabc;
#endif
    if (blk_ptr->prediction_mode_flag == INTER_MODE && candidate_ptr->is_compound)
    {
        blk_ptr->interinter_comp.type = candidate_ptr->interinter_comp.type;
        blk_ptr->interinter_comp.mask_type = candidate_ptr->interinter_comp.mask_type;
        blk_ptr->interinter_comp.wedge_index = candidate_ptr->interinter_comp.wedge_index;
        blk_ptr->interinter_comp.wedge_sign = candidate_ptr->interinter_comp.wedge_sign;
        blk_ptr->compound_idx = candidate_ptr->compound_idx;
        blk_ptr->comp_group_idx = candidate_ptr->comp_group_idx;
        if (blk_ptr->interinter_comp.type == COMPOUND_AVERAGE){
            if (blk_ptr->comp_group_idx != 0 || blk_ptr->compound_idx != 1)
                SVT_LOG("Error: Compound combination not allowed\n");
        }
    }
    blk_ptr->is_interintra_used          = candidate_ptr->is_interintra_used;
    blk_ptr->interintra_mode             = candidate_ptr->interintra_mode;
    blk_ptr->use_wedge_interintra        = candidate_ptr->use_wedge_interintra;
    blk_ptr->interintra_wedge_index      = candidate_ptr->interintra_wedge_index;

    // Set the PU level variables
    blk_ptr->interp_filters = candidate_ptr->interp_filters;
    {
        PredictionUnit       *pu_ptr = blk_ptr->prediction_unit_array;
        if (blk_ptr->prediction_mode_flag == INTRA_MODE)
        {
            blk_ptr->filter_intra_mode= candidate_ptr->filter_intra_mode;
            pu_ptr->is_directional_mode_flag = candidate_ptr->is_directional_mode_flag;
            pu_ptr->angle_delta[PLANE_TYPE_Y] = candidate_ptr->angle_delta[PLANE_TYPE_Y];

            pu_ptr->cfl_alpha_idx = candidate_ptr->cfl_alpha_idx;
            pu_ptr->cfl_alpha_signs = candidate_ptr->cfl_alpha_signs;

            pu_ptr->intra_chroma_mode = candidate_ptr->intra_chroma_mode;
            pu_ptr->is_directional_chroma_mode_flag = candidate_ptr->is_directional_chroma_mode_flag;
            pu_ptr->angle_delta[PLANE_TYPE_UV] = candidate_ptr->angle_delta[PLANE_TYPE_UV];
        }
        if (blk_ptr->prediction_mode_flag == INTRA_MODE)
        {
#if MEM_OPT_PALETTE
            if (candidate_ptr->palette_info)
                memcpy(&blk_ptr->palette_info.pmi, &candidate_ptr->palette_info->pmi, sizeof(PaletteModeInfo));
            else
                memset(&blk_ptr->palette_info.pmi, 0, sizeof(PaletteModeInfo));
#else
            eb_memcpy(&blk_ptr->palette_info.pmi, &candidate_ptr->palette_info.pmi, sizeof(PaletteModeInfo));
#endif
#if SHUT_PALETTE_BC_PD_PASS_0_1
            if (svt_av1_allow_palette(context_ptr->md_palette_level, context_ptr->blk_geom->bsize))
#else
            if(svt_av1_allow_palette(context_ptr->sb_ptr->pcs_ptr->parent_pcs_ptr->palette_mode, context_ptr->blk_geom->bsize))
#endif
#if MEM_OPT_PALETTE
            {
               if (candidate_ptr->palette_info)
                   memcpy(blk_ptr->palette_info.color_idx_map, candidate_ptr->palette_info->color_idx_map, MAX_PALETTE_SQUARE);
               else
                   memset(blk_ptr->palette_info.color_idx_map, 0, MAX_PALETTE_SQUARE);
            }
#else
               eb_memcpy(blk_ptr->palette_info.color_idx_map, candidate_ptr->palette_info.color_idx_map, MAX_PALETTE_SQUARE);
#endif
        }
        else {
            blk_ptr->palette_info.pmi.palette_size[0] = blk_ptr->palette_info.pmi.palette_size[1] = 0;
        }
        // Inter Prediction
        pu_ptr->inter_pred_direction_index = candidate_ptr->prediction_direction[0];
        context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds].merge_flag = candidate_ptr->merge_flag;
#if SB_MEM_OPT
        if (blk_ptr->prediction_mode_flag != INTER_MODE && blk_ptr->use_intrabc == 0)
#else
        if (blk_ptr->prediction_mode_flag != INTER_MODE && blk_ptr->av1xd->use_intrabc == 0)
#endif
        {
            pu_ptr->inter_pred_direction_index = 0x03;
            context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds].merge_flag = EB_FALSE;
        }
        pu_ptr->mv[REF_LIST_0].x = 0;
        pu_ptr->mv[REF_LIST_0].y = 0;

        pu_ptr->mv[REF_LIST_1].x = 0;
        pu_ptr->mv[REF_LIST_1].y = 0;

        blk_ptr->pred_mode = candidate_ptr->pred_mode;
        blk_ptr->drl_index = candidate_ptr->drl_index;

        pu_ptr->inter_mode = candidate_ptr->inter_mode;
        pu_ptr->is_compound = candidate_ptr->is_compound;
        context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds].compound_idx = candidate_ptr->compound_idx;
        context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds].interinter_comp = candidate_ptr->interinter_comp;
        pu_ptr->ref_frame_type = candidate_ptr->ref_frame_type;
        context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds].ref_frame_index_l0 = candidate_ptr->ref_frame_index_l0;
        context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds].ref_frame_index_l1 = candidate_ptr->ref_frame_index_l1;
        if (pu_ptr->inter_pred_direction_index == UNI_PRED_LIST_0)
        {
            //eb_memcpy(&pu_ptr->mv[REF_LIST_0].x,&candidate_ptr->mvs_l0,4);
            pu_ptr->mv[REF_LIST_0].x = candidate_ptr->motion_vector_xl0;
            pu_ptr->mv[REF_LIST_0].y = candidate_ptr->motion_vector_yl0;
        }

        if (pu_ptr->inter_pred_direction_index == UNI_PRED_LIST_1)
        {
            //eb_memcpy(&pu_ptr->mv[REF_LIST_1].x,&candidate_ptr->mvs_l1,4);
            pu_ptr->mv[REF_LIST_1].x = candidate_ptr->motion_vector_xl1;
            pu_ptr->mv[REF_LIST_1].y = candidate_ptr->motion_vector_yl1;
        }

        if (pu_ptr->inter_pred_direction_index == BI_PRED)
        {
            //eb_memcpy(&pu_ptr->mv[REF_LIST_0].x,&candidate_ptr->mvs,8);
            pu_ptr->mv[REF_LIST_0].x = candidate_ptr->motion_vector_xl0;
            pu_ptr->mv[REF_LIST_0].y = candidate_ptr->motion_vector_yl0;
            pu_ptr->mv[REF_LIST_1].x = candidate_ptr->motion_vector_xl1;
            pu_ptr->mv[REF_LIST_1].y = candidate_ptr->motion_vector_yl1;
        }
        if (pu_ptr->inter_pred_direction_index == UNI_PRED_LIST_0) {
            blk_ptr->predmv[0].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_0];
            blk_ptr->predmv[0].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_0];
        }
        else if (pu_ptr->inter_pred_direction_index == UNI_PRED_LIST_1) {
            blk_ptr->predmv[0].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_1];
            blk_ptr->predmv[0].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_1];
        }
        else if (pu_ptr->inter_pred_direction_index == BI_PRED) {
            blk_ptr->predmv[0].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_0];
            blk_ptr->predmv[0].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_0];
            blk_ptr->predmv[1].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_1];
            blk_ptr->predmv[1].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_1];
        }
        pu_ptr->overlappable_neighbors[0] = context_ptr->blk_ptr->prediction_unit_array[0].overlappable_neighbors[0];
        pu_ptr->overlappable_neighbors[1] = context_ptr->blk_ptr->prediction_unit_array[0].overlappable_neighbors[1];
        pu_ptr->motion_mode = candidate_ptr->motion_mode;
        pu_ptr->num_proj_ref = candidate_ptr->num_proj_ref;
        if (pu_ptr->motion_mode == WARPED_CAUSAL) {
            eb_memcpy(&context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds].wm_params_l0, &candidate_ptr->wm_params_l0, sizeof(EbWarpedMotionParams));
            eb_memcpy(&context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds].wm_params_l1, &candidate_ptr->wm_params_l1, sizeof(EbWarpedMotionParams));
        }
    }

    uint32_t txb_itr;
    uint32_t txb_index;
    uint32_t tu_total_count;
    uint32_t cu_size_log2 = context_ptr->cu_size_log2;
    tu_total_count = context_ptr->blk_geom->txb_count[blk_ptr->tx_depth];
    txb_index = 0;
    txb_itr = 0;
#if NO_ENCDEC
    int32_t txb_1d_offset = 0, txb_1d_offset_uv = 0;

    blk_ptr->block_has_coeff = 0;
#endif

    //blk_ptr->forceSmallTu = candidate_ptr->forceSmallTu;

    // Set TU
    do {
        TransformUnit *txb_ptr = &blk_ptr->txb_array[txb_index];
        context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds].y_has_coeff[txb_index] = (EbBool)(((candidate_ptr->y_has_coeff) & (1 << txb_index)) > 0);
        context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds].u_has_coeff[txb_index] = (EbBool)(((candidate_ptr->u_has_coeff) & (1 << txb_index)) > 0);
        context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds].v_has_coeff[txb_index] = (EbBool)(((candidate_ptr->v_has_coeff) & (1 << txb_index)) > 0);
        txb_ptr->transform_type[PLANE_TYPE_Y] = candidate_ptr->transform_type[txb_index];
        txb_ptr->transform_type[PLANE_TYPE_UV] = candidate_ptr->transform_type_uv;

        blk_ptr->quantized_dc[0][txb_index] = candidate_ptr->quantized_dc[0][txb_index];

#if NO_ENCDEC

        if (context_ptr->blk_geom->has_uv) {
            blk_ptr->block_has_coeff |= txb_ptr->y_has_coeff;
            blk_ptr->block_has_coeff |= txb_ptr->u_has_coeff;
            blk_ptr->block_has_coeff |= txb_ptr->v_has_coeff;
        }
        else
            blk_ptr->block_has_coeff |= txb_ptr->y_has_coeff;
        blk_ptr->cand_buff_index = lowest_cost_index;

        blk_ptr->skip_flag = 0;   //SKIP is turned OFF for this case!!
        txb_ptr->nz_coef_count[0] = candidate_ptr->eob[0][txb_index];
        txb_ptr->nz_coef_count[1] = candidate_ptr->eob[1][txb_index];
        txb_ptr->nz_coef_count[2] = candidate_ptr->eob[2][txb_index];

        if (pu_ptr->inter_pred_direction_index == UNI_PRED_LIST_0) {
            blk_ptr->predmv[0].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_0];
            blk_ptr->predmv[0].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_0];
        }
        else if (pu_ptr->inter_pred_direction_index == UNI_PRED_LIST_1) {
            blk_ptr->predmv[0].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_1];
            blk_ptr->predmv[0].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_1];
        }
        else if (pu_ptr->inter_pred_direction_index == BI_PRED) {
            blk_ptr->predmv[0].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_0];
            blk_ptr->predmv[0].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_0];
            blk_ptr->predmv[1].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_1];
            blk_ptr->predmv[1].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_1];
        }
#endif
#if NO_ENCDEC
        //copy coeff
        {
            uint32_t  bwidth = context_ptr->blk_geom->tx_width[txb_itr] < 64 ? context_ptr->blk_geom->tx_width[txb_itr] : 32;
            uint32_t  bheight = context_ptr->blk_geom->tx_height[txb_itr] < 64 ? context_ptr->blk_geom->tx_height[txb_itr] : 32;

            int32_t* src_ptr = &(((int32_t*)buffer_ptr_array[lowest_cost_index]->residual_quant_coeff_ptr->buffer_y)[txb_1d_offset]);
            int32_t* dst_ptr = &(((int32_t*)context_ptr->blk_ptr->coeff_tmp->buffer_y)[txb_1d_offset]);

            uint32_t j;

            for (j = 0; j < bheight; j++)
                eb_memcpy(dst_ptr + j * bwidth, src_ptr + j * bwidth, bwidth * sizeof(int32_t));
            if (context_ptr->blk_geom->has_uv)
            {
                // Cb
                bwidth = context_ptr->blk_geom->tx_width_uv[txb_itr];
                bheight = context_ptr->blk_geom->tx_height_uv[txb_itr];

                src_ptr = &(((int32_t*)buffer_ptr_array[lowest_cost_index]->residual_quant_coeff_ptr->buffer_cb)[txb_1d_offset_uv]);
                dst_ptr = &(((int32_t*)context_ptr->blk_ptr->coeff_tmp->buffer_cb)[txb_1d_offset_uv]);

                for (j = 0; j < bheight; j++)
                    eb_memcpy(dst_ptr + j * bwidth, src_ptr + j * bwidth, bwidth * sizeof(int32_t));
                src_ptr = &(((int32_t*)buffer_ptr_array[lowest_cost_index]->residual_quant_coeff_ptr->buffer_cr)[txb_1d_offset_uv]);
                dst_ptr = &(((int32_t*)context_ptr->blk_ptr->coeff_tmp->buffer_cr)[txb_1d_offset_uv]);

                for (j = 0; j < bheight; j++)
                    eb_memcpy(dst_ptr + j * bwidth, src_ptr + j * bwidth, bwidth * sizeof(int32_t));
            }

            txb_1d_offset += context_ptr->blk_geom->tx_width[txb_itr] * context_ptr->blk_geom->tx_height[txb_itr];
            if (context_ptr->blk_geom->has_uv)
                txb_1d_offset_uv += context_ptr->blk_geom->tx_width_uv[txb_itr] * context_ptr->blk_geom->tx_height_uv[txb_itr];
        }

#endif

        ++txb_index;
        ++txb_itr;
    } while (txb_itr < tu_total_count);
    UNUSED(cu_size_log2);
    return lowest_cost_index;
}
#if TPL_LA_LAMBDA_SCALING
uint32_t get_blk_tuned_full_lambda(struct ModeDecisionContext *context_ptr, PictureControlSet *pcs_ptr,
        uint32_t pic_full_lambda) {
    PictureParentControlSet *ppcs_ptr = pcs_ptr->parent_pcs_ptr;
    Av1Common *cm = ppcs_ptr->av1_cm;

    BlockSize bsize = context_ptr->blk_geom->bsize;
    const int bsize_base = BLOCK_16X16;
    const int num_mi_w = mi_size_wide[bsize_base];
    const int num_mi_h = mi_size_high[bsize_base];
    const int num_cols = (cm->mi_cols + num_mi_w - 1) / num_mi_w;
    const int num_rows = (cm->mi_rows + num_mi_h - 1) / num_mi_h;
    const int num_bcols = (mi_size_wide[bsize] + num_mi_w - 1) / num_mi_w;
    const int num_brows = (mi_size_high[bsize] + num_mi_h - 1) / num_mi_h;
    int mi_row = context_ptr->blk_origin_y / 4;
    int mi_col = context_ptr->blk_origin_x / 4;
    int row, col;
    double base_block_count = 0.0;
    double geom_mean_of_scale = 0.0;
    for (row = mi_row / num_mi_w;
            row < num_rows && row < mi_row / num_mi_w + num_brows; ++row) {
        for (col = mi_col / num_mi_h;
                col < num_cols && col < mi_col / num_mi_h + num_bcols; ++col) {
            const int index = row * num_cols + col;
            geom_mean_of_scale += log(ppcs_ptr->tpl_sb_rdmult_scaling_factors[index]);
            base_block_count += 1.0;
        }
    }
    geom_mean_of_scale = exp(geom_mean_of_scale / base_block_count);
    uint32_t new_full_lambda = (uint32_t)((double)pic_full_lambda * geom_mean_of_scale + 0.5);
    new_full_lambda = AOMMAX(new_full_lambda, 0);
    return new_full_lambda;
}

#if TPL_LAMBDA_IMP
void set_tuned_blk_lambda(struct ModeDecisionContext *context_ptr, PictureControlSet *pcs_ptr) {
    context_ptr->full_lambda_md[EB_8_BIT_MD] =
        get_blk_tuned_full_lambda(context_ptr,
            pcs_ptr,
            context_ptr->enc_dec_context_ptr->pic_full_lambda[EB_8_BIT_MD]);

    context_ptr->full_lambda_md[EB_10_BIT_MD] =
        get_blk_tuned_full_lambda(context_ptr,
            pcs_ptr,
            context_ptr->enc_dec_context_ptr->pic_full_lambda[EB_10_BIT_MD]);
    context_ptr->fast_lambda_md[EB_8_BIT_MD] =
        get_blk_tuned_full_lambda(context_ptr,
            pcs_ptr,
            context_ptr->enc_dec_context_ptr->pic_fast_lambda[EB_8_BIT_MD]);

    context_ptr->fast_lambda_md[EB_10_BIT_MD] =
        get_blk_tuned_full_lambda(context_ptr,
            pcs_ptr,
            context_ptr->enc_dec_context_ptr->pic_fast_lambda[EB_10_BIT_MD]);
}
#endif
#endif
