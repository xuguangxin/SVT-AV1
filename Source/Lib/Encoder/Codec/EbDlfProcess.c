/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

/*
* Copyright (c) 2016, Alliance for Open Media. All rights reserved
*
* This source code is subject to the terms of the BSD 2 Clause License and
* the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
* was not distributed with this source code in the LICENSE file, you can
* obtain it at www.aomedia.org/license/software. If the Alliance for Open
* Media Patent License 1.0 was not distributed with this source code in the
* PATENTS file, you can obtain it at www.aomedia.org/license/patent.
*/

#include <stdlib.h>
#include "EbEncHandle.h"
#include "EbDlfProcess.h"
#include "EbEncDecResults.h"
#include "EbReferenceObject.h"
#include "EbDeblockingFilter.h"
#include "EbDefinitions.h"
#include "EbSequenceControlSet.h"
#include "EbPictureControlSet.h"

void eb_av1_loop_restoration_save_boundary_lines(const Yv12BufferConfig *frame, Av1Common *cm,
                                                 int32_t after_cdef);

static void dlf_context_dctor(EbPtr p) {
    EbThreadContext *thread_context_ptr = (EbThreadContext *)p;
    DlfContext *     obj                = (DlfContext *)thread_context_ptr->priv;
    EB_DELETE(obj->temp_lf_recon_picture_ptr);
    EB_DELETE(obj->temp_lf_recon_picture16bit_ptr);
    EB_FREE_ARRAY(obj);
}
/******************************************************
 * Dlf Context Constructor
 ******************************************************/
EbErrorType dlf_context_ctor(EbThreadContext *thread_context_ptr, const EbEncHandle *enc_handle_ptr,
                             int index) {
    const SequenceControlSet *scs_ptr = enc_handle_ptr->scs_instance_array[0]->scs_ptr;
    EbBool        is_16bit     = (EbBool)(scs_ptr->static_config.encoder_bit_depth > EB_8BIT);
    EbColorFormat color_format = scs_ptr->static_config.encoder_color_format;

    DlfContext *context_ptr;
    EB_CALLOC_ARRAY(context_ptr, 1);
    thread_context_ptr->priv  = context_ptr;
    thread_context_ptr->dctor = dlf_context_dctor;

    // Input/Output System Resource Manager FIFOs
    context_ptr->dlf_input_fifo_ptr =
        eb_system_resource_get_consumer_fifo(enc_handle_ptr->enc_dec_results_resource_ptr, index);
    context_ptr->dlf_output_fifo_ptr =
        eb_system_resource_get_producer_fifo(enc_handle_ptr->dlf_results_resource_ptr, index);

    context_ptr->temp_lf_recon_picture16bit_ptr = (EbPictureBufferDesc *)EB_NULL;
    context_ptr->temp_lf_recon_picture_ptr      = (EbPictureBufferDesc *)EB_NULL;
    EbPictureBufferDescInitData temp_lf_recon_desc_init_data;
    temp_lf_recon_desc_init_data.max_width          = (uint16_t)scs_ptr->max_input_luma_width;
    temp_lf_recon_desc_init_data.max_height         = (uint16_t)scs_ptr->max_input_luma_height;
    temp_lf_recon_desc_init_data.buffer_enable_mask = PICTURE_BUFFER_DESC_FULL_MASK;

    temp_lf_recon_desc_init_data.left_padding  = PAD_VALUE;
    temp_lf_recon_desc_init_data.right_padding = PAD_VALUE;
    temp_lf_recon_desc_init_data.top_padding   = PAD_VALUE;
    temp_lf_recon_desc_init_data.bot_padding   = PAD_VALUE;

    temp_lf_recon_desc_init_data.split_mode   = EB_FALSE;
    temp_lf_recon_desc_init_data.color_format = color_format;

    if (scs_ptr->static_config.encoder_16bit_pipeline || is_16bit) {
        temp_lf_recon_desc_init_data.bit_depth = EB_16BIT;
        EB_NEW(context_ptr->temp_lf_recon_picture16bit_ptr,
               eb_recon_picture_buffer_desc_ctor,
               (EbPtr)&temp_lf_recon_desc_init_data);
    } else {
        temp_lf_recon_desc_init_data.bit_depth = EB_8BIT;
        EB_NEW(context_ptr->temp_lf_recon_picture_ptr,
               eb_recon_picture_buffer_desc_ctor,
               (EbPtr)&temp_lf_recon_desc_init_data);
    }

    return EB_ErrorNone;
}

/******************************************************
 * Dlf Kernel
 ******************************************************/
void *dlf_kernel(void *input_ptr) {
    // Context & SCS & PCS
    EbThreadContext *   thread_context_ptr = (EbThreadContext *)input_ptr;
    DlfContext *        context_ptr        = (DlfContext *)thread_context_ptr->priv;
    PictureControlSet * pcs_ptr;
    SequenceControlSet *scs_ptr;

    //// Input
    EbObjectWrapper *enc_dec_results_wrapper_ptr;
    EncDecResults *  enc_dec_results_ptr;

    //// Output
    EbObjectWrapper *  dlf_results_wrapper_ptr;
    struct DlfResults *dlf_results_ptr;

    // SB Loop variables
    for (;;) {
        // Get EncDec Results
        eb_get_full_object(context_ptr->dlf_input_fifo_ptr, &enc_dec_results_wrapper_ptr);
        EB_CHECK_END_OBJ(enc_dec_results_wrapper_ptr);

        enc_dec_results_ptr = (EncDecResults *)enc_dec_results_wrapper_ptr->object_ptr;
        pcs_ptr             = (PictureControlSet *)enc_dec_results_ptr->pcs_wrapper_ptr->object_ptr;
        scs_ptr             = (SequenceControlSet *)pcs_ptr->scs_wrapper_ptr->object_ptr;

        EbBool is_16bit = (EbBool)(scs_ptr->static_config.encoder_bit_depth > EB_8BIT);

        // TODO: remove the copy when entire 16bit pipeline is ready
        if (scs_ptr->static_config.encoder_16bit_pipeline &&
            scs_ptr->static_config.encoder_bit_depth == EB_8BIT) {
            EbPictureBufferDesc *recon_buffer, *recon_buffer_8bit;
            if (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE) {
                recon_buffer = ((EbReferenceObject *)
                    pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                    ->reference_picture16bit;
                recon_buffer_8bit = ((EbReferenceObject *)
                    pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                    ->reference_picture;
            } else {
                recon_buffer = pcs_ptr->recon_picture16bit_ptr;
                recon_buffer_8bit = pcs_ptr->recon_picture_ptr;
            }
            //copy recon from 8bit to 16bit
            uint8_t*  recon_8bit;
            int32_t   recon_stride_8bit;
            uint16_t* recon_16bit;
            int32_t   recon_stride_16bit;
            // Y
            recon_16bit = (uint16_t*)(recon_buffer->buffer_y)
                        + recon_buffer->origin_x
                        + recon_buffer->origin_y * recon_buffer->stride_y;
            recon_stride_16bit = recon_buffer->stride_y;
            recon_8bit  = recon_buffer_8bit->buffer_y
                        + recon_buffer_8bit->origin_x
                        + recon_buffer_8bit->origin_y * recon_buffer_8bit->stride_y;
            recon_stride_8bit = recon_buffer_8bit->stride_y;
            for (int j = 0; j < recon_buffer->height; j++) {
                for (int i = 0; i < recon_buffer->width; i++) {
                    recon_16bit[i + j * recon_stride_16bit] =
                        (uint16_t)recon_8bit[i + j * recon_stride_8bit];
                }
            }
            // Cb
            recon_16bit = (uint16_t*)(recon_buffer->buffer_cb)
                        + recon_buffer->origin_x / 2
                        + recon_buffer->origin_y / 2 * recon_buffer->stride_cb;
            recon_stride_16bit = recon_buffer->stride_cb;
            recon_8bit  = recon_buffer_8bit->buffer_cb
                        + recon_buffer_8bit->origin_x / 2
                        + recon_buffer_8bit->origin_y / 2 * recon_buffer_8bit->stride_cb;
            recon_stride_8bit = recon_buffer_8bit->stride_cb;
            for (int j = 0; j < recon_buffer->height / 2; j++) {
                for (int i = 0; i < recon_buffer->width / 2; i++) {
                    recon_16bit[i + j * recon_stride_16bit] =
                        (uint16_t)recon_8bit[i + j * recon_stride_8bit];
                }
            }
            // Cr
            recon_16bit = (uint16_t*)(recon_buffer->buffer_cr)
                        + recon_buffer->origin_x / 2
                        + recon_buffer->origin_y / 2 * recon_buffer->stride_cr;
            recon_stride_16bit = recon_buffer->stride_cr;
            recon_8bit  = recon_buffer_8bit->buffer_cr
                        + recon_buffer_8bit->origin_x / 2
                        + recon_buffer_8bit->origin_y / 2 * recon_buffer_8bit->stride_cr;
            recon_stride_8bit = recon_buffer_8bit->stride_cr;
            for (int j = 0; j < recon_buffer->height / 2; j++) {
                for (int i = 0; i < recon_buffer->width / 2; i++) {
                    recon_16bit[i + j * recon_stride_16bit] =
                        (uint16_t)recon_8bit[i + j * recon_stride_8bit];
                }
            }

            // //copy input from 8bit to 16bit
            uint8_t*  input_8bit;
            int32_t   input_stride_8bit;
            uint16_t* input_16bit;
            int32_t   input_stride_16bit;
            EbPictureBufferDesc* input_buffer_8bit = (EbPictureBufferDesc *)
                pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr;
            EbPictureBufferDesc* input_buffer = (EbPictureBufferDesc*)pcs_ptr->input_frame16bit;
            // Y
            input_16bit = (uint16_t*)(input_buffer->buffer_y)
                        + input_buffer->origin_x
                        + input_buffer->origin_y * input_buffer->stride_y;
            input_stride_16bit = input_buffer->stride_y;
            input_8bit  = input_buffer_8bit->buffer_y
                        + input_buffer_8bit->origin_x
                        + input_buffer_8bit->origin_y * input_buffer_8bit->stride_y;
            input_stride_8bit = input_buffer_8bit->stride_y;
            for (int j = 0; j < input_buffer->height; j++) {
                for (int i = 0; i < input_buffer->width; i++) {
                    input_16bit[i + j * input_stride_16bit] =
                        (uint16_t)input_8bit[i + j * input_stride_8bit];
                }
            }
            // Cb
            input_16bit = (uint16_t*)(input_buffer->buffer_cb)
                        + input_buffer->origin_x / 2
                        + input_buffer->origin_y / 2 * input_buffer->stride_cb;
            input_stride_16bit = input_buffer->stride_cb;
            input_8bit  = input_buffer_8bit->buffer_cb
                        + input_buffer_8bit->origin_x / 2
                        + input_buffer_8bit->origin_y / 2 * input_buffer_8bit->stride_cb;
            input_stride_8bit = input_buffer_8bit->stride_cb;
            for (int j = 0; j < input_buffer->height / 2; j++) {
                for (int i = 0; i < input_buffer->width / 2; i++) {
                    input_16bit[i + j * input_stride_16bit] =
                        (uint16_t)input_8bit[i + j * input_stride_8bit];
                }
            }
            // Cr
            input_16bit = (uint16_t*)(input_buffer->buffer_cr)
                        + input_buffer->origin_x / 2
                        + input_buffer->origin_y / 2 * input_buffer->stride_cr;
            input_stride_16bit = input_buffer->stride_cr;
            input_8bit  = input_buffer_8bit->buffer_cr
                        + input_buffer_8bit->origin_x / 2
                        + input_buffer_8bit->origin_y / 2 * input_buffer_8bit->stride_cr;
            input_stride_8bit = input_buffer_8bit->stride_cr;
            for (int j = 0; j < input_buffer->height / 2; j++) {
                for (int i = 0; i < input_buffer->width / 2; i++) {
                    input_16bit[i + j * input_stride_16bit] =
                        (uint16_t)input_8bit[i + j * input_stride_8bit];
                }
            }
        }

        EbBool dlf_enable_flag = (EbBool)pcs_ptr->parent_pcs_ptr->loop_filter_mode;
#if TILES_PARALLEL
        uint16_t total_tile_cnt = pcs_ptr->parent_pcs_ptr->av1_cm->tiles_info.tile_cols *
                                  pcs_ptr->parent_pcs_ptr->av1_cm->tiles_info.tile_rows;
        // Jing: Move sb level lf to here if tile_parallel
        if ((dlf_enable_flag && pcs_ptr->parent_pcs_ptr->loop_filter_mode >= 2) ||
            (dlf_enable_flag && pcs_ptr->parent_pcs_ptr->loop_filter_mode == 1 &&
             total_tile_cnt > 1)) {
#else
        if (dlf_enable_flag && pcs_ptr->parent_pcs_ptr->loop_filter_mode >= 2) {
#endif
            EbPictureBufferDesc *recon_buffer =
                is_16bit ? pcs_ptr->recon_picture16bit_ptr : pcs_ptr->recon_picture_ptr;

            if (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE) {
                if (scs_ptr->static_config.encoder_16bit_pipeline || is_16bit)
                    recon_buffer =
                        ((EbReferenceObject *)
                             pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                            ->reference_picture16bit;
                else
                    recon_buffer =
                        ((EbReferenceObject *)
                             pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                            ->reference_picture;
            } else {
                recon_buffer = scs_ptr->static_config.encoder_16bit_pipeline ||
                    is_16bit ? pcs_ptr->recon_picture16bit_ptr : pcs_ptr->recon_picture_ptr;
            }
            eb_av1_loop_filter_init(pcs_ptr);

            if (pcs_ptr->parent_pcs_ptr->loop_filter_mode == 2) {
                eb_av1_pick_filter_level(
                    context_ptr,
                    (EbPictureBufferDesc *)pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr,
                    pcs_ptr,
                    LPF_PICK_FROM_Q);
            }

            eb_av1_pick_filter_level(
                context_ptr,
                (EbPictureBufferDesc *)pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr,
                pcs_ptr,
                LPF_PICK_FROM_FULL_IMAGE);

#if NO_ENCDEC
            //NO DLF
            pcs_ptr->parent_pcs_ptr->lf.filter_level[0] = 0;
            pcs_ptr->parent_pcs_ptr->lf.filter_level[1] = 0;
            pcs_ptr->parent_pcs_ptr->lf.filter_level_u  = 0;
            pcs_ptr->parent_pcs_ptr->lf.filter_level_v  = 0;
#endif
            eb_av1_loop_filter_frame(recon_buffer, pcs_ptr, 0, 3);
        }

        // TODO: remove the copy when entire 16bit pipeline is ready
        if (scs_ptr->static_config.encoder_16bit_pipeline &&
            scs_ptr->static_config.encoder_bit_depth == EB_8BIT &&
            !scs_ptr->seq_header.enable_restoration &&
            !scs_ptr->seq_header.enable_cdef) {
            EbPictureBufferDesc *recon_buffer, *recon_buffer_8bit;
            if (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE) {
                recon_buffer = ((EbReferenceObject *)
                    pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                    ->reference_picture16bit;
                recon_buffer_8bit = ((EbReferenceObject *)
                    pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                    ->reference_picture;
            } else {
                recon_buffer = pcs_ptr->recon_picture16bit_ptr;
                recon_buffer_8bit = pcs_ptr->recon_picture_ptr;
            }
            //copy recon from 16bit to 8bit
            uint8_t*  recon_8bit;
            int32_t   recon_stride_8bit;
            uint16_t* recon_16bit;
            int32_t   recon_stride_16bit;
            // Y
            recon_16bit = (uint16_t*)(recon_buffer->buffer_y)
                        + recon_buffer->origin_x
                        + recon_buffer->origin_y * recon_buffer->stride_y;
            recon_stride_16bit = recon_buffer->stride_y;
            recon_8bit  = recon_buffer_8bit->buffer_y
                        + recon_buffer_8bit->origin_x
                        + recon_buffer_8bit->origin_y * recon_buffer_8bit->stride_y;
            recon_stride_8bit = recon_buffer_8bit->stride_y;
            for (int j = 0; j < recon_buffer->height; j++) {
                for (int i = 0; i < recon_buffer->width; i++) {
                    recon_8bit[i + j * recon_stride_8bit] =
                        (uint8_t)recon_16bit[i + j * recon_stride_16bit];
                }
            }
            // Cb
            recon_16bit = (uint16_t*)(recon_buffer->buffer_cb)
                        + recon_buffer->origin_x / 2
                        + recon_buffer->origin_y / 2 * recon_buffer->stride_cb;
            recon_stride_16bit = recon_buffer->stride_cb;
            recon_8bit  = recon_buffer_8bit->buffer_cb
                        + recon_buffer_8bit->origin_x / 2
                        + recon_buffer_8bit->origin_y / 2 * recon_buffer_8bit->stride_cb;
            recon_stride_8bit = recon_buffer_8bit->stride_cb;
            for (int j = 0; j < recon_buffer->height / 2; j++) {
                for (int i = 0; i < recon_buffer->width / 2; i++) {
                    recon_8bit[i + j * recon_stride_8bit] =
                        (uint8_t)recon_16bit[i + j * recon_stride_16bit];
                }
            }
            // Cr
            recon_16bit = (uint16_t*)(recon_buffer->buffer_cr)
                        + recon_buffer->origin_x / 2
                        + recon_buffer->origin_y / 2 * recon_buffer->stride_cr;
            recon_stride_16bit = recon_buffer->stride_cr;
            recon_8bit  = recon_buffer_8bit->buffer_cr
                        + recon_buffer_8bit->origin_x / 2
                        + recon_buffer_8bit->origin_y / 2 * recon_buffer_8bit->stride_cr;
            recon_stride_8bit = recon_buffer_8bit->stride_cr;
            for (int j = 0; j < recon_buffer->height / 2; j++) {
                for (int i = 0; i < recon_buffer->width / 2; i++) {
                    recon_8bit[i + j * recon_stride_8bit] =
                        (uint8_t)recon_16bit[i + j * recon_stride_16bit];
                }
            }
        }

        //pre-cdef prep
        {
            Av1Common *          cm = pcs_ptr->parent_pcs_ptr->av1_cm;
            EbPictureBufferDesc *recon_picture_ptr;
            if (is_16bit) {
                if (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE)
                    recon_picture_ptr =
                        ((EbReferenceObject *)
                             pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                            ->reference_picture16bit;
                else
                    recon_picture_ptr = pcs_ptr->recon_picture16bit_ptr;
            } else {
                if (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE)
                    recon_picture_ptr =
                        ((EbReferenceObject *)
                             pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                            ->reference_picture;
                else
                    recon_picture_ptr = pcs_ptr->recon_picture_ptr;
            }
            if (scs_ptr->static_config.encoder_16bit_pipeline) {
                if (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE) {
                    recon_picture_ptr = ((EbReferenceObject *)
                        pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                        ->reference_picture16bit;
                } else {
                    recon_picture_ptr = pcs_ptr->recon_picture16bit_ptr;
                }
                cm->use_highbitdepth = 1;
            }
            link_eb_to_aom_buffer_desc(recon_picture_ptr, cm->frame_to_show);
            if (scs_ptr->seq_header.enable_restoration)
                eb_av1_loop_restoration_save_boundary_lines(cm->frame_to_show, cm, 0);
            if (scs_ptr->seq_header.enable_cdef && pcs_ptr->parent_pcs_ptr->cdef_filter_mode) {
                if (scs_ptr->static_config.encoder_16bit_pipeline || is_16bit) {
                    pcs_ptr->src[0] = (uint16_t *)recon_picture_ptr->buffer_y +
                                      (recon_picture_ptr->origin_x +
                                       recon_picture_ptr->origin_y * recon_picture_ptr->stride_y);
                    pcs_ptr->src[1] =
                        (uint16_t *)recon_picture_ptr->buffer_cb +
                        (recon_picture_ptr->origin_x / 2 +
                         recon_picture_ptr->origin_y / 2 * recon_picture_ptr->stride_cb);
                    pcs_ptr->src[2] =
                        (uint16_t *)recon_picture_ptr->buffer_cr +
                        (recon_picture_ptr->origin_x / 2 +
                         recon_picture_ptr->origin_y / 2 * recon_picture_ptr->stride_cr);

                    EbPictureBufferDesc *input_picture_ptr = pcs_ptr->input_frame16bit;
                    pcs_ptr->ref_coeff[0] =
                        (uint16_t *)input_picture_ptr->buffer_y +
                        (input_picture_ptr->origin_x +
                         input_picture_ptr->origin_y * input_picture_ptr->stride_y);
                    pcs_ptr->ref_coeff[1] =
                        (uint16_t *)input_picture_ptr->buffer_cb +
                        (input_picture_ptr->origin_x / 2 +
                         input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_cb);
                    pcs_ptr->ref_coeff[2] =
                        (uint16_t *)input_picture_ptr->buffer_cr +
                        (input_picture_ptr->origin_x / 2 +
                         input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_cr);
                } else {
                    EbByte rec_ptr =
                        &((recon_picture_ptr->buffer_y)[recon_picture_ptr->origin_x +
                                                        recon_picture_ptr->origin_y *
                                                            recon_picture_ptr->stride_y]);
                    EbByte rec_ptr_cb =
                        &((recon_picture_ptr->buffer_cb)[recon_picture_ptr->origin_x / 2 +
                                                         recon_picture_ptr->origin_y / 2 *
                                                             recon_picture_ptr->stride_cb]);
                    EbByte rec_ptr_cr =
                        &((recon_picture_ptr->buffer_cr)[recon_picture_ptr->origin_x / 2 +
                                                         recon_picture_ptr->origin_y / 2 *
                                                             recon_picture_ptr->stride_cr]);

                    EbPictureBufferDesc *input_picture_ptr =
                        (EbPictureBufferDesc *)pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr;
                    EbByte enh_ptr =
                        &((input_picture_ptr->buffer_y)[input_picture_ptr->origin_x +
                                                        input_picture_ptr->origin_y *
                                                            input_picture_ptr->stride_y]);
                    EbByte enh_ptr_cb =
                        &((input_picture_ptr->buffer_cb)[input_picture_ptr->origin_x / 2 +
                                                         input_picture_ptr->origin_y / 2 *
                                                             input_picture_ptr->stride_cb]);
                    EbByte enh_ptr_cr =
                        &((input_picture_ptr->buffer_cr)[input_picture_ptr->origin_x / 2 +
                                                         input_picture_ptr->origin_y / 2 *
                                                             input_picture_ptr->stride_cr]);

                    pcs_ptr->src[0] = (uint16_t *)rec_ptr;
                    pcs_ptr->src[1] = (uint16_t *)rec_ptr_cb;
                    pcs_ptr->src[2] = (uint16_t *)rec_ptr_cr;

                    pcs_ptr->ref_coeff[0] = (uint16_t *)enh_ptr;
                    pcs_ptr->ref_coeff[1] = (uint16_t *)enh_ptr_cb;
                    pcs_ptr->ref_coeff[2] = (uint16_t *)enh_ptr_cr;
                }
            }
        }

        pcs_ptr->cdef_segments_column_count = scs_ptr->cdef_segment_column_count;
        pcs_ptr->cdef_segments_row_count    = scs_ptr->cdef_segment_row_count;
        pcs_ptr->cdef_segments_total_count =
            (uint16_t)(pcs_ptr->cdef_segments_column_count * pcs_ptr->cdef_segments_row_count);
        pcs_ptr->tot_seg_searched_cdef = 0;
        uint32_t segment_index;

        for (segment_index = 0; segment_index < pcs_ptr->cdef_segments_total_count;
             ++segment_index) {
            // Get Empty DLF Results to Cdef
            eb_get_empty_object(context_ptr->dlf_output_fifo_ptr, &dlf_results_wrapper_ptr);
            dlf_results_ptr = (struct DlfResults *)dlf_results_wrapper_ptr->object_ptr;
            dlf_results_ptr->pcs_wrapper_ptr = enc_dec_results_ptr->pcs_wrapper_ptr;
            dlf_results_ptr->segment_index   = segment_index;
            // Post DLF Results
            eb_post_full_object(dlf_results_wrapper_ptr);
        }

        // Release EncDec Results
        eb_release_object(enc_dec_results_wrapper_ptr);
    }

    return EB_NULL;
}
