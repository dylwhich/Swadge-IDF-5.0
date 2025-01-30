#pragma once

#include <stddef.h>

#include "hashMap.h"

typedef bool (*loadFn_t)(const char* tag, void* data);
typedef void (*unloadFn_t)(void* data);

typedef struct
{
    hashMap_t map;

    size_t structSize;
    loadFn_t loadFn;
    unloadFn_t unloadFn;
} loader_t;

void loaderInit(loader_t* loader, size_t dataSize, loadFn_t loadFn, unloadFn_t unloadFn);
void loaderDeinit(loader_t* loader);
const void* loaderGet(loader_t* loader, const char* key);
void loaderReturn(loader_t* loader, const void* value);
