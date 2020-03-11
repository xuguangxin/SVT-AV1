/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#ifndef EbPredictionUnit_h
#define EbPredictionUnit_h

#include "EbDefinitions.h"
#include "EbSyntaxElements.h"
#include "EbDefinitions.h"
#include "EbMotionVectorUnit.h"
#ifdef __cplusplus
extern "C" {
#endif
#pragma pack(push, 1)
#if CLEAN_UP_SB_DATA
typedef struct PredictionUnit {
    Mv mv[MAX_NUM_OF_REF_PIC_LIST]; //ec
#if CLEAN_UP_SB_DATA_7
    uint8_t inter_pred_direction_index; // ec
#else
    Mvd      mvd[MAX_NUM_OF_REF_PIC_LIST]; // 16-bytes
    unsigned merge_flag : 1; // to move
    unsigned inter_pred_direction_index : 2; // ec
    unsigned intra_luma_mode : 6;

    unsigned intra_luma_left_mode : 6;
    unsigned intra_luma_top_mode : 6;

    uint8_t intra_chroma_left_mode;
    uint8_t intra_chroma_top_mode;
#endif
    // Intra Mode
    int32_t  angle_delta[PLANE_TYPES]; // ec
    EbBool is_directional_mode_flag; // ec
    EbBool is_directional_chroma_mode_flag; // ec
    uint32_t intra_chroma_mode; // ec

    // Inter Mode
    PredictionMode inter_mode; // ec
    EbBool is_compound; // ec
#if !CLEAN_UP_SB_DATA_10
    uint8_t                compound_idx;
    InterInterCompoundData interinter_comp;
#endif
#if !CLEAN_UP_SB_DATA_7
    uint32_t pred_mv_weight;
#endif
    uint8_t ref_frame_type; // ec
#if !CLEAN_UP_SB_DATA_10
    int8_t ref_frame_index_l0;
    int8_t ref_frame_index_l1;
#endif
#if !CLEAN_UP_SB_DATA_7
    uint8_t ref_mv_index;
    EbBool  is_new_mv;
    EbBool  is_zero_mv;
#endif
    MotionMode motion_mode; // ec
    uint16_t num_proj_ref; // ec
#if !CLEAN_UP_SB_DATA_10
    EbWarpedMotionParams wm_params_l0;
    EbWarpedMotionParams wm_params_l1;
#endif
    uint32_t overlappable_neighbors[2]; // ec
    // Index of the alpha Cb and alpha Cr combination
    int32_t cfl_alpha_idx; // ec
    // Joint sign of alpha Cb and alpha Cr
    int32_t cfl_alpha_signs; // ec
} PredictionUnit;
#else
typedef struct PredictionUnit {
    Mv       mv[MAX_NUM_OF_REF_PIC_LIST]; // 16-bytes
    Mvd      mvd[MAX_NUM_OF_REF_PIC_LIST]; // 16-bytes
    unsigned merge_flag : 1;
    unsigned inter_pred_direction_index : 2;
    unsigned intra_luma_mode : 6;
    unsigned intra_luma_left_mode : 6;
    unsigned intra_luma_top_mode : 6;

    uint8_t intra_chroma_left_mode;
    uint8_t intra_chroma_top_mode;
    // Intra Mode
    int32_t  angle_delta[PLANE_TYPES];
    EbBool   is_directional_mode_flag;
    EbBool   is_directional_chroma_mode_flag;
    uint32_t intra_chroma_mode;

    // Inter Mode
    PredictionMode         inter_mode;
    EbBool                 is_compound;
    uint8_t                compound_idx;
    InterInterCompoundData interinter_comp;

    uint32_t pred_mv_weight;
    uint8_t  ref_frame_type;
    int8_t   ref_frame_index_l0;
    int8_t   ref_frame_index_l1;
    uint8_t  ref_mv_index;
    EbBool   is_new_mv;
    EbBool   is_zero_mv;

    MotionMode           motion_mode;
    uint16_t             num_proj_ref;
    EbWarpedMotionParams wm_params_l0;
    EbWarpedMotionParams wm_params_l1;
    uint32_t             overlappable_neighbors[2];

    // Index of the alpha Cb and alpha Cr combination
    int32_t cfl_alpha_idx;
    // Joint sign of alpha Cb and alpha Cr
    int32_t cfl_alpha_signs;
} PredictionUnit;
#endif
#pragma pack(pop)

#ifdef __cplusplus
}
#endif
#endif //EbPredictionUnit_h
