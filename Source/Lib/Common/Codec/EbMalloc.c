/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/
#include <stdio.h>
#include <inttypes.h>


#include "EbMalloc.h"
#include "EbThreads.h"

#ifdef DEBUG_MEMORY_USAGE

static EbHandle g_malloc_mutex;

#ifdef _WIN32

#include <windows.h>

static INIT_ONCE g_malloc_once = INIT_ONCE_STATIC_INIT;

BOOL CALLBACK create_malloc_mutex (
    PINIT_ONCE InitOnce,
    PVOID Parameter,
    PVOID *lpContext)
{
    (void)InitOnce;
    (void)Parameter;
    (void)lpContext;
    g_malloc_mutex = eb_create_mutex();
    return TRUE;
}

static EbHandle get_malloc_mutex()
{
    InitOnceExecuteOnce(&g_malloc_once, create_malloc_mutex, NULL, NULL);
    return g_malloc_mutex;
}
#elif defined(__linux__) || defined(__APPLE__)
#include <pthread.h>
static void create_malloc_mutex()
{
    g_malloc_mutex = eb_create_mutex();
}

static pthread_once_t g_malloc_once = PTHREAD_ONCE_INIT;

static EbHandle get_malloc_mutex()
{
    pthread_once(&g_malloc_once, create_malloc_mutex);
    return g_malloc_mutex;
}
#endif // _WIN32

//hash function to speedup etnry search
uint32_t hash(void* p)
{
#define MASK32 ((((uint64_t)1)<<32)-1)

    uint64_t v = (uint64_t)p;
    uint64_t low32 = v & MASK32;
    return (uint32_t)((v >> 32) + low32);
}

typedef struct MemoryEntry{
    void* ptr;
    EbPtrType type;
    size_t count;
    const char* file;
    uint32_t line;
} MemoryEntry;

//+1 to get a better hash result
#define MEM_ENTRY_SIZE (32 * 1024 * 1024 + 1)

MemoryEntry g_mem_entry[MEM_ENTRY_SIZE];

#define TO_INDEX(v) ((v) % MEM_ENTRY_SIZE)
static EbBool g_add_mem_entry_warning = EB_TRUE;
static EbBool g_remove_mem_entry_warning = EB_TRUE;


/*********************************************************************************
*
* @brief
*  compare and update current memory entry.
*
* @param[in] e
*  current memory entry.
*
* @param[in] param
*  param you set to for_each_mem_entry
*
*
* @returns  return EB_TRUE if you want get early exit in for_each_mem_entry
*
s*
********************************************************************************/

typedef EbBool (*Predicate)(MemoryEntry* e, void* param);

/*********************************************************************************
*
* @brief
*  Loop through mem entries.
*
* @param[in] start
*  loop start position
*
* @param[in] pred
*  return EB_TRUE if you want early exit
*
* @param[out] param
*  param send to pred.
*
* @returns  return EB_TRUE if we got early exit.
*
* @remarks
*  Any remarks
*
********************************************************************************/

static EbBool for_each_mem_entry(uint32_t start, Predicate pred, void* param)
{

    uint32_t s = TO_INDEX(start);
    uint32_t i = s;

    EbHandle m = get_malloc_mutex();
    eb_block_on_mutex(m);

    do {
        MemoryEntry* e = g_mem_entry + i;
        if (pred(e, param)) {
            eb_release_mutex(m);
            return EB_TRUE;
        }
        i++;
        i = TO_INDEX(i);
    } while (i != s);
    eb_release_mutex(m);
    return EB_FALSE;
}

static const char* mem_type_name(EbPtrType type)
{
    static const char *name[EB_PTR_TYPE_TOTAL] = {"memory", "aligned memory", "mutex", "semaphore", "thread"};
    return name[type];
}

static EbBool add_mem_entry(MemoryEntry* e, void* param)
{
    MemoryEntry* new_item = (MemoryEntry*)param;
    if (!e->ptr) {
        EB_MEMCPY(e, new_item, sizeof(MemoryEntry));
        return EB_TRUE;
    }
    return EB_FALSE;
}


void eb_add_mem_entry(void* ptr,  EbPtrType type, size_t count, const char* file, uint32_t line)
{
    MemoryEntry item;
    item.ptr = ptr;
    item.type = type;
    item.count = count;
    item.file = file;
    item.line = line;
    if (for_each_mem_entry(hash(ptr), add_mem_entry, &item))
        return;
    if (g_add_mem_entry_warning) {
        fprintf(stderr, "SVT: can't add memory entry.\r\n");
        fprintf(stderr, "SVT: You have memory leak or you need increase MEM_ENTRY_SIZE\r\n");
        g_add_mem_entry_warning = EB_FALSE;
    }
}

static EbBool remove_mem_entry(MemoryEntry* e, void* param)
{
    MemoryEntry* item = (MemoryEntry*)param;
    if (e->ptr == item->ptr && e->type == item->type) {
        e->ptr = NULL;
        return EB_TRUE;
    }
    return EB_FALSE;
}

void eb_remove_mem_entry(void* ptr, EbPtrType type)
{
    if (!ptr)
        return;
    MemoryEntry item;
    item.ptr = ptr;
    item.type = type;
    if (for_each_mem_entry(hash(ptr), remove_mem_entry, &item))
        return;
    if (g_remove_mem_entry_warning) {
        fprintf(stderr, "SVT: something wrong. you freed a unallocated memory %p, type = %s\r\n", ptr, mem_type_name(type));
        g_remove_mem_entry_warning = EB_FALSE;
    }
}

typedef struct MemSummary {
    uint64_t amount[EB_PTR_TYPE_TOTAL];
    uint32_t occupied;
} MemSummary;

static EbBool count_mem_entry(MemoryEntry* e, void* param)
{
    MemSummary* sum = (MemSummary*)param;
    if (e->ptr) {
        sum->amount[e->type] += e->count;
        sum->occupied++;
    }
    return EB_FALSE;
}

static void get_memory_usage_and_scale(uint64_t amount, double* usage, char* scale)
{
    char scales[] = { ' ', 'K', 'M', 'G' };
    int i;
    uint64_t v;
    for (i = 1; i < sizeof(scales); i++) {
        v = (uint64_t)1 << (i * 10);
        if (amount < v)
            break;
    }
    i--;
    v = (uint64_t)1 << (i * 10);
    *usage = (double)amount / v;
    *scale = scales[i];
}

#endif

void eb_print_memory_usage()
{
#ifdef DEBUG_MEMORY_USAGE
    MemSummary sum;
    double fulless;
    double usage;
    char scale;
    memset(&sum, 0, sizeof(MemSummary));

    for_each_mem_entry(0, count_mem_entry, &sum);
    printf("SVT Memory Usage:\r\n");
    get_memory_usage_and_scale(sum.amount[EB_N_PTR], &usage, &scale);
    printf("    allocated memory:         %.2lf %cB\r\n", usage, scale);
    get_memory_usage_and_scale(sum.amount[EB_A_PTR], &usage, &scale);
    printf("    allocated aligned memory: %.2lf %cB\r\n", usage, scale);

    printf("    mutex count: %d\r\n", (int)sum.amount[EB_MUTEX]);
    printf("    semaphore count: %d\r\n", (int)sum.amount[EB_SEMAPHORE]);
    printf("    thread count: %d\r\n", (int)sum.amount[EB_THREAD]);
    fulless = (double)sum.occupied / MEM_ENTRY_SIZE;
    printf("    hash table fulless: %f, hash bucket is %s\r\n", fulless, fulless < .3 ? "healthy":"too full" );
#endif
}

static int g_component_count;
void eb_increase_component_count()
{
#ifdef DEBUG_MEMORY_USAGE
    EbHandle m = get_malloc_mutex();
    eb_block_on_mutex(m);
    g_component_count++;
    eb_release_mutex(m);
#endif
}

#ifdef DEBUG_MEMORY_USAGE
static EbBool print_leak(MemoryEntry* e, void* param)
{
    if (e->ptr) {
        EbBool* leaked = (EbBool*)param;
        *leaked = EB_TRUE;
        fprintf(stderr, "SVt: %s leaked at %s:L%d\r\n", mem_type_name(e->type), e->file, e->line);
    }
    //loop through all items
    return EB_FALSE;
}
#endif

void eb_decrease_component_count()
{
#ifdef DEBUG_MEMORY_USAGE
    EbBool all_components_released;
    EbHandle m = get_malloc_mutex();
    eb_block_on_mutex(m);
    g_component_count--;
    all_components_released = !g_component_count;
    eb_release_mutex(m);
    if (all_components_released) {
        EbBool leaked = EB_FALSE;
        for_each_mem_entry(0, print_leak, &leaked);
        if (!leaked) {
            printf("SVT: you have no memory leak\r\n");
        }
    }
#endif
}
