#ifndef _SPRITE_H_
#define _SPRITE_H_

//==============================================================================
// Includes
//==============================================================================
#include <stdint.h>
#include "wsg.h"
#include "aabb_utils.h"

//==============================================================================
// Structs
//==============================================================================
typedef struct
{
    wsg_t wsg;
    int16_t originX;
    int16_t originY;
    box_t collisionBox;
} sprite_t;

#endif
