#define _VMATH_IMPLEMENTATION_
#include "include/vmath/vmath.h"
#include "vshape.h"
#include "assert.h"

int main() {
    printf("Sphere to Sphere: ");
    fflush(stdout);
    f32 position[] = {0, 0, 0};
    VShape first = VSHAPE_SphereCreate(position, 5);
    position[0] += 5;
    position[1] += 1;
    position[2] += 1;
    VShape second = VSHAPE_SphereCreate(position, 3);
    assert(VSHAPE_Collide(first, second));
    second.center[0] += 10;
    assert(!VSHAPE_Collide(first, second));
    printf("OK\n");


    printf("Box to Box: ");
    fflush(stdout);
    VShape boxO = VSHAPE_BoxCreate((f32[3]){0, 0, 0}, (f32[3]){1, 1, 1}, (f32[3]){0, 0, 0});
    VShape boxB = VSHAPE_BoxCreate((f32[3]){0.5, 0.5, 0.5}, (f32[3]){1, 1, 1}, (f32[3]){0, 0, 0});
    assert(VSHAPE_Collide(boxO, boxB));

    VShape box2 = VSHAPE_BoxCreate((f32[3]){1.1, 1, 1}, (f32[3]){1, 1, 1}, (f32[3]){0, 0, 0});
    assert(!VSHAPE_Collide(boxO, box2));

    float angle = 45;
    VShape box4 = VSHAPE_BoxCreate((f32[3]){1.22, 0.0, 0.0}, (f32[3]){1, 1, 1}, (f32[3]){0, V_PI*(angle/180), 0});
    VShape box5 = VSHAPE_BoxCreate((f32[3]){1.18, 0.0, 0.0}, (f32[3]){1, 1, 1}, (f32[3]){0, V_PI*(angle/180), 0});
    assert(!VSHAPE_Collide(boxO, box4));
    assert( VSHAPE_Collide(boxO, box5));
    printf("OK\n");

    printf("AABB to AABB: ");
    fflush(stdout);
    VShape aabb1 = VSHAPE_AABBCreate((f32[3]){0.1, 0.1, 0.1}, (f32[3]){2, 2, 2});
    VShape aabb2 = VSHAPE_AABBCreate((f32[3]){1.9, 1.9, 1.9}, (f32[3]){2, 2, 2});
    f32 to_s[3];
    assert(VSHAPE_CollisionGet(aabb1, aabb2, to_s));
//    printf("%f, %f, %f\n", to_s[0], to_s[1], to_s[2]);
//    assert(VM3_Eq(to_s, ((f32[]){0.2, 0.2, 0.2})));
    

    VShape aabb3 = VSHAPE_AABBCreate((f32[3]){0, 0, 0}, (f32[3]){1, 1, 1});
    VShape aabb4 = VSHAPE_AABBCreate((f32[3]){3, 2, 2}, (f32[3]){1, 1, 1});
    assert(!VSHAPE_Collide(aabb3, aabb4));

    VShape aabb5 = VSHAPE_AABBCreate((f32[3]){0, 0, 0}, (f32[3]){2, 2, 2});
    VShape aabb6 = VSHAPE_AABBCreate((f32[3]){2, 2, 2}, (f32[3]){2, 2, 2});
    assert(!VSHAPE_Collide(aabb5, aabb6));

    printf("OK\n");


    printf("Frustrum to Box: ");
    fflush(stdout);
    VShape frustrum = VSHAPE_FrustrumCreate((f32[3]){0, 0, 0}, (f32[3]){0, 0, 0}, 100, V_PI/3, 1);
    VShape object_out = VSHAPE_BoxCreate((f32[3]){5, 5, 5}, VM3_ONE, VM3_ZERO);
    VShape object_in  = VSHAPE_BoxCreate((f32[3]){0, 0,-5}, VM3_ONE, VM3_ZERO);
    assert(!VSHAPE_Collide(frustrum, object_out));
    assert(VSHAPE_Collide(frustrum, object_in));
    
    printf("OK\n");
}
