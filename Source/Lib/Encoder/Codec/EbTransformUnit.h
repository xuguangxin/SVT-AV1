/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#ifndef EbTransformUnit_h
#define EbTransformUnit_h

#include "EbDefinitions.h"
#ifdef __cplusplus
extern "C" {
#endif
#if CLEAN_UP_SB_DATA_7
#define TRANSFORM_UNIT_MAX_COUNT 16
#pragma pack(push, 1)
typedef struct TransformUnit {
#if !CLEAN_UP_SB_DATA_8
    uint8_t  u_has_coeff : 1;
    uint8_t  v_has_coeff : 1;
    uint8_t  y_has_coeff : 1;
#endif
    uint16_t nz_coef_count[3];
    TxType   transform_type[PLANE_TYPES];
} TransformUnit;
#else
#define TRANSFORM_UNIT_MAX_COUNT 21
#define TRANSFORM_UNIT_2Nx2N_AREA 16
#define TRANSFORM_UNIT_NxN_AREA 4

#pragma pack(push, 1)
typedef struct TransformUnit {
    unsigned txb_index : 5;
    unsigned split_flag : 1;
    unsigned u_has_coeff : 1;
    unsigned v_has_coeff : 1;
    unsigned y_has_coeff : 1;
    uint16_t nz_coef_count[3];
    EbBool   is_only_dc[3];
    TxType   transform_type[PLANE_TYPES];
} TransformUnit;
#endif
#pragma pack(pop)
#ifdef __cplusplus
}
#endif
#endif // EbTransformUnit_h
