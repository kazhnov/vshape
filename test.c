#define _VMATH_IMPLEMENTATION_
#include "include/vmath/vmath.h"
#include "vshape.h"
#include "assert.h"

int main() {
    f32 position[] = {0, 0, 0};
    VShape first = VSHAPE_SphereCreate(position, 5);
    position[0] += 5;
    position[1] += 1;
    position[2] += 1;
    VShape second = VSHAPE_SphereCreate(position, 3);
    assert(VSHAPE_Collide(first, second));
    second.center[0] += 10;
    assert(!VSHAPE_Collide(first, second));

    VShape box1 = VSHAPE_BoxCreate((f32[3]){0, 0, 0}, (f32[3]){1, 1, 1}, (f32[3]){0, 0, 0});
    VShape box2 = VSHAPE_BoxCreate((f32[3]){1, 0, 1}, (f32[3]){1, 1, 1}, (f32[3]){0, V_PI/6, 0});
    assert(VSHAPE_Collide(box1, box2));

    VShape box3 = VSHAPE_BoxCreate((f32[3]){0, 0, 0}, (f32[3]){1, 1, 1}, (f32[3]){0, 0, 0});
    VShape box4 = VSHAPE_BoxCreate((f32[3]){1, 1, 1}, (f32[3]){1, 1, 1}, (f32[3]){0, V_PI/6, 0});
    assert(!VSHAPE_Collide(box3, box4));

    
    VShape aabb1 = VSHAPE_AABBCreate((f32[3]){0, 0, 0}, (f32[3]){1, 1, 1});
    VShape aabb2 = VSHAPE_AABBCreate((f32[3]){2, 2, 2}, (f32[3]){1, 1, 1});
    assert(VSHAPE_Collide(aabb1, aabb2));

    VShape aabb3 = VSHAPE_AABBCreate((f32[3]){0, 0, 0}, (f32[3]){1, 1, 1});
    VShape aabb4 = VSHAPE_AABBCreate((f32[3]){3, 2, 2}, (f32[3]){1, 1, 1});
    assert(!VSHAPE_Collide(aabb3, aabb4));


}
