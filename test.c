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

    VM3_Set(position, 0, 0, 0);
    VShape box1 = VSHAPE_BoxCreate(position, (f32[]){1, 1, 1}, (f32[3]){0, 0, 0});
    VM3_Set(position, 1., 0., 1.);
    VShape box2 = VSHAPE_BoxCreate(position, (f32[]){1, 1, 1}, (f32[3]){0, V_PI/6, 0});
    assert(VSHAPE_Collide(box1, box2));
}
