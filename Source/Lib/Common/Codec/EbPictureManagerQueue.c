/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#include <stdlib.h>
#include "EbPictureManagerQueue.h"

EbErrorType input_queue_entry_ctor(
    InputQueueEntry      *entryPtr)
{
    (void)entryPtr;
    return EB_ErrorNone;
}

EbErrorType reference_queue_entry_ctor(
    ReferenceQueueEntry  **entry_dbl_ptr)
{
    ReferenceQueueEntry *entryPtr;
    EB_ALLOC_OBJECT(ReferenceQueueEntry*, entryPtr, sizeof(ReferenceQueueEntry), EB_N_PTR);
    *entry_dbl_ptr = entryPtr;

    entryPtr->reference_object_ptr = (EbObjectWrapper*)EB_NULL;
    entryPtr->picture_number = ~0u;
    entryPtr->dependent_count = 0;
    entryPtr->reference_available = EB_FALSE;

    EB_MALLOC(int32_t*, entryPtr->list0.list, sizeof(int32_t) * (1 << MAX_TEMPORAL_LAYERS), EB_N_PTR);

    EB_MALLOC(int32_t*, entryPtr->list1.list, sizeof(int32_t) * (1 << MAX_TEMPORAL_LAYERS), EB_N_PTR);

    return EB_ErrorNone;
}
