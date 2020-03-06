/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#include <stdlib.h>

#include "EbSystemResourceManager.h"
#include "EbDefinitions.h"
#include "EbThreads.h"

static void eb_fifo_dctor(EbPtr p) {
    EbFifo *obj = (EbFifo *)p;
    EB_DESTROY_SEMAPHORE(obj->counting_semaphore);
    EB_DESTROY_MUTEX(obj->lockout_mutex);
}
/**************************************
 * eb_fifo_ctor
 **************************************/
static EbErrorType eb_fifo_ctor(EbFifo *fifoPtr, uint32_t initial_count, uint32_t max_count,
                                EbObjectWrapper *firstWrapperPtr, EbObjectWrapper *lastWrapperPtr,
                                EbMuxingQueue *queue_ptr) {
    fifoPtr->dctor = eb_fifo_dctor;
    // Create Counting Semaphore
    EB_CREATE_SEMAPHORE(fifoPtr->counting_semaphore, initial_count, max_count);

    // Create Buffer Pool Mutex
    EB_CREATE_MUTEX(fifoPtr->lockout_mutex);

    // Initialize Fifo First & Last ptrs
    fifoPtr->first_ptr = firstWrapperPtr;
    fifoPtr->last_ptr  = lastWrapperPtr;

    // Copy the Muxing Queue ptr this Fifo belongs to
    fifoPtr->queue_ptr = queue_ptr;

    return EB_ErrorNone;
}

/**************************************
 * eb_fifo_push_back
 **************************************/
static EbErrorType eb_fifo_push_back(EbFifo *fifoPtr, EbObjectWrapper *wrapper_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    // If FIFO is empty
    if (fifoPtr->first_ptr == (EbObjectWrapper *)EB_NULL) {
        fifoPtr->first_ptr = wrapper_ptr;
        fifoPtr->last_ptr  = wrapper_ptr;
    } else {
        fifoPtr->last_ptr->next_ptr = wrapper_ptr;
        fifoPtr->last_ptr           = wrapper_ptr;
    }

    fifoPtr->last_ptr->next_ptr = (EbObjectWrapper *)EB_NULL;

    return return_error;
}

/**************************************
 * eb_fifo_pop_front
 **************************************/
static EbErrorType eb_fifo_pop_front(EbFifo *fifoPtr, EbObjectWrapper **wrapper_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    // Set wrapper_ptr to head of BufferPool
    *wrapper_ptr = fifoPtr->first_ptr;

    // Update tail of BufferPool if the BufferPool is now empty
    fifoPtr->last_ptr =
        (fifoPtr->first_ptr == fifoPtr->last_ptr) ? (EbObjectWrapper *)EB_NULL : fifoPtr->last_ptr;

    // Update head of BufferPool
    fifoPtr->first_ptr = fifoPtr->first_ptr->next_ptr;

    return return_error;
}

static void eb_circular_buffer_dctor(EbPtr p) {
    EbCircularBuffer *obj = (EbCircularBuffer *)p;
    EB_FREE(obj->array_ptr);
}

/**************************************
 * eb_circular_buffer_ctor
 **************************************/
static EbErrorType eb_circular_buffer_ctor(EbCircularBuffer *bufferPtr,
                                           uint32_t          buffer_total_count) {
    bufferPtr->dctor = eb_circular_buffer_dctor;

    bufferPtr->buffer_total_count = buffer_total_count;

    EB_CALLOC(bufferPtr->array_ptr, bufferPtr->buffer_total_count, sizeof(EbPtr));

    return EB_ErrorNone;
}

/**************************************
 * eb_circular_buffer_empty_check
 **************************************/
static EbBool eb_circular_buffer_empty_check(EbCircularBuffer *bufferPtr) {
    return ((bufferPtr->head_index == bufferPtr->tail_index) &&
            (bufferPtr->array_ptr[bufferPtr->head_index] == EB_NULL))
               ? EB_TRUE
               : EB_FALSE;
}

/**************************************
 * eb_circular_buffer_pop_front
 **************************************/
static EbErrorType eb_circular_buffer_pop_front(EbCircularBuffer *bufferPtr, EbPtr *object_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    // Copy the head of the buffer into the object_ptr
    *object_ptr                                 = bufferPtr->array_ptr[bufferPtr->head_index];
    bufferPtr->array_ptr[bufferPtr->head_index] = EB_NULL;

    // Increment the head & check for rollover
    bufferPtr->head_index = (bufferPtr->head_index == bufferPtr->buffer_total_count - 1)
                                ? 0
                                : bufferPtr->head_index + 1;

    // Decrement the Current Count
    --bufferPtr->current_count;

    return return_error;
}

/**************************************
 * eb_circular_buffer_push_back
 **************************************/
static EbErrorType eb_circular_buffer_push_back(EbCircularBuffer *bufferPtr, EbPtr object_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    // Copy the pointer into the array
    bufferPtr->array_ptr[bufferPtr->tail_index] = object_ptr;

    // Increment the tail & check for rollover
    bufferPtr->tail_index = (bufferPtr->tail_index == bufferPtr->buffer_total_count - 1)
                                ? 0
                                : bufferPtr->tail_index + 1;

    // Increment the Current Count
    ++bufferPtr->current_count;

    return return_error;
}

/**************************************
 * eb_circular_buffer_push_front
 **************************************/
static EbErrorType eb_circular_buffer_push_front(EbCircularBuffer *bufferPtr, EbPtr object_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    // Decrement the head_index
    bufferPtr->head_index = (bufferPtr->head_index == 0) ? bufferPtr->buffer_total_count - 1
                                                         : bufferPtr->head_index - 1;

    // Copy the pointer into the array
    bufferPtr->array_ptr[bufferPtr->head_index] = object_ptr;

    // Increment the Current Count
    ++bufferPtr->current_count;

    return return_error;
}

void eb_muxing_queue_dctor(EbPtr p) {
    EbMuxingQueue *obj = (EbMuxingQueue *)p;
    EB_DELETE_PTR_ARRAY(obj->process_fifo_ptr_array, obj->process_total_count);
    EB_DELETE(obj->object_queue);
    EB_DELETE(obj->process_queue);
    EB_DESTROY_MUTEX(obj->lockout_mutex);
}

/**************************************
 * eb_muxing_queue_ctor
 **************************************/
static EbErrorType eb_muxing_queue_ctor(EbMuxingQueue *queue_ptr, uint32_t object_total_count,
                                        uint32_t process_total_count) {
    uint32_t    process_index;
    EbErrorType return_error = EB_ErrorNone;

    queue_ptr->dctor               = eb_muxing_queue_dctor;
    queue_ptr->process_total_count = process_total_count;

    // Lockout Mutex
    EB_CREATE_MUTEX(queue_ptr->lockout_mutex);

    // Construct Object Circular Buffer
    EB_NEW(queue_ptr->object_queue, eb_circular_buffer_ctor, object_total_count);
    // Construct Process Circular Buffer
    EB_NEW(queue_ptr->process_queue, eb_circular_buffer_ctor, queue_ptr->process_total_count);
    // Construct the Process Fifos
    EB_ALLOC_PTR_ARRAY(queue_ptr->process_fifo_ptr_array, queue_ptr->process_total_count);

    for (process_index = 0; process_index < queue_ptr->process_total_count; ++process_index) {
        EB_NEW(queue_ptr->process_fifo_ptr_array[process_index],
               eb_fifo_ctor,
               0,
               object_total_count,
               (EbObjectWrapper *)EB_NULL,
               (EbObjectWrapper *)EB_NULL,
               queue_ptr);
    }

    return return_error;
}

/**************************************
 * eb_muxing_queue_assignation
 **************************************/
static EbErrorType eb_muxing_queue_assignation(EbMuxingQueue *queue_ptr) {
    EbErrorType      return_error = EB_ErrorNone;
    EbFifo *         process_fifo_ptr;
    EbObjectWrapper *wrapper_ptr;

    // while loop
    while ((eb_circular_buffer_empty_check(queue_ptr->object_queue) == EB_FALSE) &&
           (eb_circular_buffer_empty_check(queue_ptr->process_queue) == EB_FALSE)) {
        // Get the next process
        eb_circular_buffer_pop_front(queue_ptr->process_queue, (void **)&process_fifo_ptr);

        // Get the next object
        eb_circular_buffer_pop_front(queue_ptr->object_queue, (void **)&wrapper_ptr);

        // Block on the Process Fifo's Mutex
        eb_block_on_mutex(process_fifo_ptr->lockout_mutex);

        // Put the object on the fifo
        eb_fifo_push_back(process_fifo_ptr, wrapper_ptr);

        // Release the Process Fifo's Mutex
        eb_release_mutex(process_fifo_ptr->lockout_mutex);

        // Post the semaphore
        eb_post_semaphore(process_fifo_ptr->counting_semaphore);
    }

    return return_error;
}

/**************************************
 * eb_muxing_queue_object_push_back
 **************************************/
static EbErrorType eb_muxing_queue_object_push_back(EbMuxingQueue *  queue_ptr,
                                                    EbObjectWrapper *object_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    eb_circular_buffer_push_back(queue_ptr->object_queue, object_ptr);

    eb_muxing_queue_assignation(queue_ptr);

    return return_error;
}

/**************************************
* eb_muxing_queue_object_push_front
**************************************/
static EbErrorType eb_muxing_queue_object_push_front(EbMuxingQueue *  queue_ptr,
                                                     EbObjectWrapper *object_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    eb_circular_buffer_push_front(queue_ptr->object_queue, object_ptr);

    eb_muxing_queue_assignation(queue_ptr);

    return return_error;
}

static EbFifo *eb_muxing_queue_get_fifo(EbMuxingQueue *queue_ptr, uint32_t index) {
    assert(queue_ptr->process_fifo_ptr_array && (queue_ptr->process_total_count > index));
    return queue_ptr->process_fifo_ptr_array[index];
}

/*********************************************************************
 * eb_object_release_enable
 *   Enables the release_enable member of EbObjectWrapper.  Used by
 *   certain objects (e.g. SequenceControlSet) to control whether
 *   EbObjectWrappers are allowed to be released or not.
 *
 *   resource_ptr
 *      pointer to the SystemResource that manages the EbObjectWrapper.
 *      The emptyFifo's lockout_mutex is used to write protect the
 *      modification of the EbObjectWrapper.
 *
 *   wrapper_ptr
 *      pointer to the EbObjectWrapper to be modified.
 *********************************************************************/
EbErrorType eb_object_release_enable(EbObjectWrapper *wrapper_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    eb_block_on_mutex(wrapper_ptr->system_resource_ptr->empty_queue->lockout_mutex);

    wrapper_ptr->release_enable = EB_TRUE;

    eb_release_mutex(wrapper_ptr->system_resource_ptr->empty_queue->lockout_mutex);

    return return_error;
}

/*********************************************************************
 * eb_object_release_disable
 *   Disables the release_enable member of EbObjectWrapper.  Used by
 *   certain objects (e.g. SequenceControlSet) to control whether
 *   EbObjectWrappers are allowed to be released or not.
 *
 *   resource_ptr
 *      pointer to the SystemResource that manages the EbObjectWrapper.
 *      The emptyFifo's lockout_mutex is used to write protect the
 *      modification of the EbObjectWrapper.
 *
 *   wrapper_ptr
 *      pointer to the EbObjectWrapper to be modified.
 *********************************************************************/
EbErrorType eb_object_release_disable(EbObjectWrapper *wrapper_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    eb_block_on_mutex(wrapper_ptr->system_resource_ptr->empty_queue->lockout_mutex);

    wrapper_ptr->release_enable = EB_FALSE;

    eb_release_mutex(wrapper_ptr->system_resource_ptr->empty_queue->lockout_mutex);

    return return_error;
}

/*********************************************************************
 * eb_object_inc_live_count
 *   Increments the live_count member of EbObjectWrapper.  Used by
 *   certain objects (e.g. SequenceControlSet) to count the number of active
 *   pointers of a EbObjectWrapper in pipeline at any point in time.
 *
 *   resource_ptr
 *      pointer to the SystemResource that manages the EbObjectWrapper.
 *      The emptyFifo's lockout_mutex is used to write protect the
 *      modification of the EbObjectWrapper.
 *
 *   wrapper_ptr
 *      pointer to the EbObjectWrapper to be modified.
 *********************************************************************/
EbErrorType eb_object_inc_live_count(EbObjectWrapper *wrapper_ptr, uint32_t increment_number) {
    EbErrorType return_error = EB_ErrorNone;

    eb_block_on_mutex(wrapper_ptr->system_resource_ptr->empty_queue->lockout_mutex);

    wrapper_ptr->live_count += increment_number;

    eb_release_mutex(wrapper_ptr->system_resource_ptr->empty_queue->lockout_mutex);

    return return_error;
}

//ugly hack
typedef struct DctorAble {
    EbDctor dctor;
} DctorAble;

void eb_object_wrapper_dctor(EbPtr p) {
    EbObjectWrapper *wrapper = (EbObjectWrapper *)p;
    if (wrapper->object_destroyer) {
        //customized destoryer
        if (wrapper->object_ptr) wrapper->object_destroyer(wrapper->object_ptr);
    } else {
        //hack....
        DctorAble *obj = (DctorAble *)wrapper->object_ptr;
        EB_DELETE(obj);
    }
}

static EbErrorType eb_object_wrapper_ctor(EbObjectWrapper *wrapper, EbSystemResource *resource,
                                          EbCreator object_creator, EbPtr object_init_data_ptr,
                                          EbDctor object_destroyer) {
    EbErrorType ret;

    wrapper->dctor = eb_object_wrapper_dctor;
    ret            = object_creator(&wrapper->object_ptr, object_init_data_ptr);
    if (ret != EB_ErrorNone) return ret;
    wrapper->release_enable      = EB_TRUE;
    wrapper->quit_signal         = EB_FALSE;
    wrapper->system_resource_ptr = resource;
    wrapper->object_destroyer    = object_destroyer;
    return EB_ErrorNone;
}

static void eb_system_resource_dctor(EbPtr p) {
    EbSystemResource *obj = (EbSystemResource *)p;
    EB_DELETE(obj->full_queue);
    EB_DELETE(obj->empty_queue);
    EB_DELETE_PTR_ARRAY(obj->wrapper_ptr_pool, obj->object_total_count);
}

/*********************************************************************
 * eb_system_resource_ctor
 *   Constructor for EbSystemResource.  Fully constructs all members
 *   of EbSystemResource including the object with the passed
 *   object_ctor function.
 *
 *   resource_ptr
 *     pointer that will contain the SystemResource to be constructed.
 *
 *   object_total_count
 *     Number of objects to be managed by the SystemResource.
 *
 *   object_ctor
 *     Function pointer to the constructor of the object managed by
 *     SystemResource referenced by resource_ptr. No object level
 *     construction is performed if object_ctor is NULL.
 *
 *   object_init_data_ptr

 *     pointer to data block to be used during the construction of
 *     the object. object_init_data_ptr is passed to object_ctor when
 *     object_ctor is called.
 *   object_destroyer
 *     object destroyer, will call dctor if this is null
 *********************************************************************/
EbErrorType eb_system_resource_ctor(EbSystemResource *resource_ptr, uint32_t object_total_count,
                                    uint32_t producer_process_total_count,
                                    uint32_t consumer_process_total_count, EbCreator object_creator,
                                    EbPtr object_init_data_ptr, EbDctor object_destroyer) {
    uint32_t    wrapper_index;
    EbErrorType return_error = EB_ErrorNone;
    resource_ptr->dctor      = eb_system_resource_dctor;

    resource_ptr->object_total_count = object_total_count;

    // Allocate array for wrapper pointers
    EB_ALLOC_PTR_ARRAY(resource_ptr->wrapper_ptr_pool, resource_ptr->object_total_count);

    // Initialize each wrapper
    for (wrapper_index = 0; wrapper_index < resource_ptr->object_total_count; ++wrapper_index) {
        EB_NEW(resource_ptr->wrapper_ptr_pool[wrapper_index],
               eb_object_wrapper_ctor,
               resource_ptr,
               object_creator,
               object_init_data_ptr,
               object_destroyer);
    }

    // Initialize the Empty Queue
    EB_NEW(resource_ptr->empty_queue,
           eb_muxing_queue_ctor,
           resource_ptr->object_total_count,
           producer_process_total_count);
    // Fill the Empty Fifo with every ObjectWrapper
    for (wrapper_index = 0; wrapper_index < resource_ptr->object_total_count; ++wrapper_index) {
        eb_muxing_queue_object_push_back(resource_ptr->empty_queue,
                                         resource_ptr->wrapper_ptr_pool[wrapper_index]);
    }

    // Initialize the Full Queue
    if (consumer_process_total_count) {
        EB_NEW(resource_ptr->full_queue,
               eb_muxing_queue_ctor,
               resource_ptr->object_total_count,
               consumer_process_total_count);
    } else {
        resource_ptr->full_queue = (EbMuxingQueue *)EB_NULL;
    }

    return return_error;
}

EbFifo *eb_system_resource_get_producer_fifo(const EbSystemResource *resource_ptr, uint32_t index) {
    return eb_muxing_queue_get_fifo(resource_ptr->empty_queue, index);
}

EbFifo *eb_system_resource_get_consumer_fifo(const EbSystemResource *resource_ptr, uint32_t index) {
    return eb_muxing_queue_get_fifo(resource_ptr->full_queue, index);
}

EbErrorType eb_system_resource_shutdown(const EbSystemResource *resource_ptr) {
    EbFifo *fifo_ptr = eb_system_resource_get_producer_fifo(resource_ptr, 0);
    EbObjectWrapper *wrapper_ptr;
    //notify all consumers we are shutdown
    for (unsigned int i = 0; i < resource_ptr->full_queue->process_total_count; i++) {
        eb_get_empty_object(fifo_ptr, &wrapper_ptr);
        wrapper_ptr->quit_signal = EB_TRUE;
        eb_post_full_object(wrapper_ptr);
    }
    return EB_ErrorNone;
}

/*********************************************************************
 * EbSystemResourceReleaseProcess
 *********************************************************************/
static EbErrorType eb_release_process(EbFifo *process_fifo_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    eb_block_on_mutex(process_fifo_ptr->queue_ptr->lockout_mutex);

    eb_circular_buffer_push_front(process_fifo_ptr->queue_ptr->process_queue, process_fifo_ptr);

    eb_muxing_queue_assignation(process_fifo_ptr->queue_ptr);

    eb_release_mutex(process_fifo_ptr->queue_ptr->lockout_mutex);

    return return_error;
}

/*********************************************************************
 * EbSystemResourcePostObject
 *   Queues a full EbObjectWrapper to the SystemResource. This
 *   function posts the SystemResource fullFifo counting_semaphore.
 *   This function is write protected by the SystemResource fullFifo
 *   lockout_mutex.
 *
 *   resource_ptr
 *      pointer to the SystemResource that the EbObjectWrapper is
 *      posted to.
 *
 *   wrapper_ptr
 *      pointer to EbObjectWrapper to be posted.
 *********************************************************************/
EbErrorType eb_post_full_object(EbObjectWrapper *object_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    eb_block_on_mutex(object_ptr->system_resource_ptr->full_queue->lockout_mutex);

    eb_muxing_queue_object_push_back(object_ptr->system_resource_ptr->full_queue, object_ptr);

    eb_release_mutex(object_ptr->system_resource_ptr->full_queue->lockout_mutex);

    return return_error;
}

/*********************************************************************
 * EbSystemResourceReleaseObject
 *   Queues an empty EbObjectWrapper to the SystemResource. This
 *   function posts the SystemResource emptyFifo counting_semaphore.
 *   This function is write protected by the SystemResource emptyFifo
 *   lockout_mutex.
 *
 *   object_ptr
 *      pointer to EbObjectWrapper to be released.
 *********************************************************************/
EbErrorType eb_release_object(EbObjectWrapper *object_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    eb_block_on_mutex(object_ptr->system_resource_ptr->empty_queue->lockout_mutex);

    // Decrement live_count
    object_ptr->live_count =
        (object_ptr->live_count == 0) ? object_ptr->live_count : object_ptr->live_count - 1;

    if ((object_ptr->release_enable == EB_TRUE) && (object_ptr->live_count == 0)) {
        // Set live_count to EB_ObjectWrapperReleasedValue
        object_ptr->live_count = EB_ObjectWrapperReleasedValue;

        eb_muxing_queue_object_push_front(object_ptr->system_resource_ptr->empty_queue, object_ptr);
    }

    eb_release_mutex(object_ptr->system_resource_ptr->empty_queue->lockout_mutex);

    return return_error;
}

/*********************************************************************
 * EbSystemResourceGetEmptyObject
 *   Dequeues an empty EbObjectWrapper from the SystemResource.  This
 *   function blocks on the SystemResource emptyFifo counting_semaphore.
 *   This function is write protected by the SystemResource emptyFifo
 *   lockout_mutex.
 *
 *   resource_ptr
 *      pointer to the SystemResource that provides the empty
 *      EbObjectWrapper.
 *
 *   wrapper_dbl_ptr
 *      Double pointer used to pass the pointer to the empty
 *      EbObjectWrapper pointer.
 *********************************************************************/
EbErrorType eb_get_empty_object(EbFifo *empty_fifo_ptr, EbObjectWrapper **wrapper_dbl_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    // Queue the Fifo requesting the empty fifo
    eb_release_process(empty_fifo_ptr);

    // Block on the counting Semaphore until an empty buffer is available
    eb_block_on_semaphore(empty_fifo_ptr->counting_semaphore);

    // Acquire lockout Mutex
    eb_block_on_mutex(empty_fifo_ptr->lockout_mutex);

    // Get the empty object
    eb_fifo_pop_front(empty_fifo_ptr, wrapper_dbl_ptr);

    // Reset the wrapper's live_count
    (*wrapper_dbl_ptr)->live_count = 0;

    // Object release enable
    (*wrapper_dbl_ptr)->release_enable = EB_TRUE;

    // Release Mutex
    eb_release_mutex(empty_fifo_ptr->lockout_mutex);

    return return_error;
}

/*********************************************************************
 * EbSystemResourceGetFullObject
 *   Dequeues an full EbObjectWrapper from the SystemResource. This
 *   function blocks on the SystemResource fullFifo counting_semaphore.
 *   This function is write protected by the SystemResource fullFifo
 *   lockout_mutex.
 *
 *   resource_ptr
 *      pointer to the SystemResource that provides the full
 *      EbObjectWrapper.
 *
 *   wrapper_dbl_ptr
 *      Double pointer used to pass the pointer to the full
 *      EbObjectWrapper pointer.
 *********************************************************************/
EbErrorType eb_get_full_object(EbFifo *full_fifo_ptr, EbObjectWrapper **wrapper_dbl_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    // Queue the Fifo requesting the full fifo
    eb_release_process(full_fifo_ptr);

    // Block on the counting Semaphore until an empty buffer is available
    eb_block_on_semaphore(full_fifo_ptr->counting_semaphore);

    // Acquire lockout Mutex
    eb_block_on_mutex(full_fifo_ptr->lockout_mutex);

    eb_fifo_pop_front(full_fifo_ptr, wrapper_dbl_ptr);

    // Release Mutex
    eb_release_mutex(full_fifo_ptr->lockout_mutex);

    return return_error;
}

/**************************************
* eb_fifo_pop_front
**************************************/
static EbBool eb_fifo_peak_front(EbFifo *fifoPtr) {
    // Set wrapper_ptr to head of BufferPool
    if (fifoPtr->first_ptr == (EbObjectWrapper *)EB_NULL)
        return EB_TRUE;
    else
        return EB_FALSE;
}

EbErrorType eb_get_full_object_non_blocking(
    EbFifo   *full_fifo_ptr,
    EbObjectWrapper **wrapper_dbl_ptr)
{
    EbErrorType return_error = EB_ErrorNone;
    EbBool      fifo_empty;
    // Queue the Fifo requesting the full fifo
    eb_release_process(full_fifo_ptr);

    // Acquire lockout Mutex
    eb_block_on_mutex(full_fifo_ptr->lockout_mutex);

    fifo_empty = eb_fifo_peak_front(full_fifo_ptr);

    // Release Mutex
    eb_release_mutex(full_fifo_ptr->lockout_mutex);

    if (fifo_empty == EB_FALSE)
        eb_get_full_object(full_fifo_ptr, wrapper_dbl_ptr);
    else
        *wrapper_dbl_ptr = (EbObjectWrapper *)EB_NULL;

    return return_error;
}
