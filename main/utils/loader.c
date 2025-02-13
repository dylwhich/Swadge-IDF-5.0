#include "loader.h"

#include <string.h>
#include <stdlib.h>
#include <stddef.h>

typedef struct
{
    char* key;
    void* value;
    int refcount;
} loaderItem_t;

void loaderInit(loader_t* loader, size_t dataSize, loadFn_t loadFn, unloadFn_t unloadFn)
{
    hashInit(&loader->map, 16);
    loader->structSize = dataSize;
    loader->loadFn = loadFn;
    loader->unloadFn = unloadFn;
}

void loaderDeinit(loader_t* loader)
{
    loaderClear(loader);
    hashDeinit(&loader->map);
}

void loaderClear(loader_t* loader)
{
    hashIterator_t iter = {0};
    while (hashIterate(&loader->map, &iter))
    {
        loaderItem_t* item = (loaderItem_t*)iter.value;

        if (!hashIterRemove(&loader->map, &iter))
        {
            break;
        }
        loader->unloadFn(item->value);
        free(item);
    }

    hashIterReset(&iter);
}

const void* loaderGet(loader_t* loader, const char* key)
{
    loaderItem_t* found = (loaderItem_t*)hashGet(&loader->map, key);
    if (!found)
    {
        // No item found, create a new container
        char tmp[loader->structSize];
        if (loader->loadFn(key, tmp))
        {
            void* data = malloc(loader->structSize + strlen(key) + 1 + sizeof(loaderItem_t));
            if (NULL != data)
            {
                loaderItem_t* item = (loaderItem_t*) data;
                void* itemData = (void*)(((char*)data) + sizeof(loaderItem_t));
                char* keyData = ((char*)data) + sizeof(loaderItem_t) + loader->structSize;

                memcpy(itemData, tmp, loader->structSize);
                strcpy(keyData, key);

                item->key = keyData;
                item->value = itemData;
                item->refcount = 0;

                hashPut(&loader->map, item->key, data);

                found = item;
            }
        }
    }

    if (found)
    {
        found->refcount++;
        return found->value;
    }

    return NULL;
}

void loaderReturn(loader_t* loader, const void* value)
{
    // ok this is some crazy shit we can do because of how the allocation is structured
    loaderItem_t* item = ((loaderItem_t*)value) - 1;

    item->refcount--;
    
    if (item->refcount <= 0)
    {
        // TODO check return value and warn if null
        hashRemove(&loader->map, item->key);

        loader->unloadFn(item->value);
        free(item);
    }
}

