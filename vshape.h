#include <stdbool.h>

typedef enum {
    VSHAPE_SPHERE,
    VSHAPE_BOX,
    VSHAPE_AABB,
    VSHAPE_MESH,
    VSHAPE_FRUSTRUM,
    
    VSHAPE_SIZE
} VShapeType;

typedef struct {
    VShapeType type;
    float center[3];
    float rotation[3];
    float size[3];
    float radius;
    int vertex_count;
    float *vertices;
} VShape;

typedef struct {

} VManifold;

VShape VSHAPE_SphereCreate(float center[3], float radius);
VShape VSHAPE_BoxCreate(float center[3], float size[3], float rotation[3]);
VShape VSHAPE_FrustrumCreate(float center[3], float distance, float fov, float ratio);


VShape VSHAPE_AABBCreate(float center[3], float size[3]);
VShape VSHAPE_AABBGet(VShape shape);

VShape VSHAPE_BoxFromVertices(void *vertices, u32 count, u32 stride, u32 offset);

bool VSHAPE_Collide(VShape a, VShape b);
bool VSHAPE_CollisionGet(VShape a, VShape b, float to_separate[3]);
bool VSHAPE_Collide(VShape a, VShape b);
