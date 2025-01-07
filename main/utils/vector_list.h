#pragma once

#include <stddef.h>

#include "linked_list.h"

typedef struct vectorListChunk
{
    /// @brief The number of items allocated in this chunk
    uint16_t length;
    /// @brief The number of items used by this chunk
    uint16_t count;
    /// @brief A pointer to the node data allocated for this chunk
    node_t* nodes;
    /// @brief A pointer to the member data allocated for this chunk
    void* datas;
    /// @brief A pointer to the next allocated vector, or NULL if this is the last one
    struct vectorListChunk* next;
} vectorListChunk_t;

typedef struct
{
    /// @brief The size of each data member struct
    size_t itemSize;

    /// @brief The number of members in each chunk
    size_t chunkCapacity;

    /// @brief The number of allocated chunks
    size_t chunkCount;

    /// @brief The number of used members
    int count;

    /// @brief Flags used to enable extra options and functionality
    int flags;

    /// @brief Pointer to the first chunk
    vectorListChunk_t* chunks;
} vectorList_t;

/**
 * @brief Struct used for iterating through a vector list efficiently
 */
typedef struct
{
    /// @brief The value of the current item
    void* value;

    /// @internal @brief Internal iterator state
    int chunk;
} vecListIterator_t;

/// @brief Flag to double the number of items in each successive chunk
#define VECLIST_OPT_GROW_EXP = (1 << 0)

/**
 * @brief Initializes a vector list for data of the given size
 * 
 * @param list A pointer to the list struct to initialize
 * @param size The size of the item this vector will hold
 * @param chunkFactor The number of items to preallocate for each chunk.
 * @param flags Bitmask of extra options for the list, or 0 for defaults
 */
void initVectorList(vectorList_t* list, size_t size, size_t chunkFactor, int flags);

void* vectorListAddNew(vectorList_t* list);
void* vectorListInsertCopy(vectorList_t* list, void* item);
void* vectorListGet(vectorList_t* list, int index);
bool vectorListIterate(const vectorList_t* list, vecListIterator_t* iterator);
void* vectorListRemove(vectorList_t* list, int index);
void* vectorListIterRemove(vectorList_t* list, vecListIterator_t* iterator);