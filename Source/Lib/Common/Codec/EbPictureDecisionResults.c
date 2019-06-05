/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#include <stdlib.h>

#include "EbPictureDecisionResults.h"

EbErrorType picture_decision_result_ctor(
    EbPtr *object_dbl_ptr,
    EbPtr object_init_data_ptr)
{
    PictureDecisionResults *object_ptr;
    EB_ALLOC_OBJECT(PictureDecisionResults *, object_ptr, sizeof(PictureDecisionResults), EB_N_PTR);

    *object_dbl_ptr = (EbPtr)object_ptr;
    object_init_data_ptr = 0;
    (void)object_init_data_ptr;

    return EB_ErrorNone;
}

EbErrorType picture_decision_result_creator(
    EbPtr *object_dbl_ptr,
    EbPtr object_init_data_ptr)
{
    return picture_decision_result_ctor(object_dbl_ptr, object_init_data_ptr);
}
