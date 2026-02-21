#include <stdbool.h>

typedef enum {
    VSHAPE_SPHERE,
    VSHAPE_BOX,
    VSHAPE_AABB,
    
    VSHAPE_SIZE
} VShapeType;

typedef struct {
    VShapeType type;
    float center[3];
    float rotation[3];
    float size[3];
    float radius;
} VShape;

typedef struct {

} VManifold;

VShape VSHAPE_SphereCreate(float center[3], float radius);
VShape VSHAPE_BoxCreate(float center[3], float size[3], float rotation[3]);

VShape VSHAPE_AABBCreate(float center[3], float size[3]);
VShape VSHAPE_AABBGet(VShape shape);

bool VSHAPE_Collide(VShape a, VShape b);
