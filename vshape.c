#include "vshape.h"
#include "include/vmath/vmath.h"
#include <assert.h>
#include <float.h>

#define MAX(a, b) (a > b ? a : b)
#define MIN(a, b) (a < b ? a : b)

bool iVSHAPE_CollisionSpheresGet(VShape a, VShape b);
bool iVSHAPE_CollisionBoxesGet(VShape a, VShape b);
bool iVSHAPE_CollisionAABBsGet(VShape a, VShape b);
void iVSHAPE_BoxCornersGet(VShape box, float corners[8][3]);

VShape VSHAPE_SphereCreate(float center[3], float radius) {
    VShape sphere = {0};
    sphere.type = VSHAPE_SPHERE;
    VM3_Copy(sphere.center, center);
    sphere.radius = radius;
    return sphere;
}

VShape VSHAPE_BoxCreate(float center[3], float size[3], float rotation[3]) {
    VShape box = {0};
    box.type = VSHAPE_BOX;
    VM3_Copy(box.center, center);
    VM3_Copy(box.size, size);
    VM3_Copy(box.rotation, rotation);
    return box;
}

VShape VSHAPE_AABBCreate(float center[3], float size[3]) {
    VShape aabb = {0};
    aabb.type = VSHAPE_AABB;
    VM3_Copy(aabb.center, center);
    VM3_Copy(aabb.size, size);
    return aabb;
}

VShape VSHAPE_AABBGet(VShape shape) {
    switch (shape.type) {
    case VSHAPE_BOX: {
	float corners[8][3];
	iVSHAPE_BoxCornersGet(shape, corners);
	float max[3]; float min[3];
	VM3_Set(max, FLT_MIN, FLT_MIN, FLT_MIN);
	VM3_Set(min, FLT_MAX, FLT_MAX, FLT_MAX);
	for (int i = 0; i < 8; i++) {
	    max[0] = MAX(corners[i][0], max[0]);
	    max[1] = MAX(corners[i][1], max[1]);
	    max[2] = MAX(corners[i][2], max[2]);

	    min[0] = MIN(corners[i][0], min[0]);
	    min[1] = MIN(corners[i][1], min[1]);
	    min[2] = MIN(corners[i][2], min[2]);
	}
	float size[3]; float center[3];
	VM3_SubtractO(max, min, size);
	VM3_Scale(size, 0.5);
       	VM3_AddO(min, size, center);
	return VSHAPE_AABBCreate(center, size);
    }
	break;
	
    case VSHAPE_SPHERE:
	return VSHAPE_AABBCreate(shape.center, (f32[3]){
		shape.radius,
		shape.radius,
		shape.radius
	    });
	break;
	
    case VSHAPE_AABB:
	return VSHAPE_AABBCreate(shape.center, shape.size);
	break;
    default:
	assert(false);
	break;
	
    }
}


bool VSHAPE_Collide(VShape a, VShape b) {
    switch (a.type) {
    case VSHAPE_SPHERE:
	switch (b.type) {
	case VSHAPE_SPHERE:
	    return iVSHAPE_CollisionSpheresGet(a, b);
	    break;
	case VSHAPE_BOX:
	    break;
	default:
	    break;
	}
	break;
    case VSHAPE_BOX:
	switch (b.type) {
	case VSHAPE_SPHERE:
	    break;
	case VSHAPE_BOX:
	    return iVSHAPE_CollisionBoxesGet(a, b);
	    break;
	default:
	    break;
	}
	break;
    case VSHAPE_AABB:
	switch (b.type) {
	case VSHAPE_SPHERE:
	    break;
	case VSHAPE_BOX:
	    break;
	case VSHAPE_AABB:
	    return iVSHAPE_CollisionAABBsGet(a, b);
	    break;
	default:
	    break;
	}
    default:
	assert(false);
	break;
    }
    assert(false);
}

bool iVSHAPE_CollisionSpheresGet(VShape a, VShape b) {
    return VM3_Distance(a.center, b.center) < a.radius + b.radius;
}

bool iVSHAPE_CollisionAABBsGet(VShape a, VShape b) {
    return
	a.center[0] - a.size[0] <= b.center[0] + b.size[0] &&
	a.center[0] + a.size[0] >= b.center[0] - b.size[0] &&
	a.center[1] - a.size[1] <= b.center[1] + b.size[1] &&
	a.center[1] + a.size[1] >= b.center[1] - b.size[1] &&
	a.center[2] - a.size[2] <= b.center[2] + b.size[2] &&
	a.center[2] + a.size[2] >= b.center[2] - b.size[2];
}

void iVSHAPE_BoxCornersGet(VShape box, float corners[8][3]) {
    float matrix[16] = VM44_IDENTITY;
    VM44_Rotate(matrix, box.rotation);
    VM44_Scale(matrix, box.size);
    VM44_Translate(matrix, box.center);
    
    VM44_V3MultiplyO(matrix, (f32[]){-0.5, -0.5, -0.5}, corners[0]);
    VM44_V3MultiplyO(matrix, (f32[]){-0.5, -0.5,  0.5}, corners[1]);
    VM44_V3MultiplyO(matrix, (f32[]){-0.5,  0.5, -0.5}, corners[2]);
    VM44_V3MultiplyO(matrix, (f32[]){-0.5,  0.5,  0.5}, corners[3]);
    VM44_V3MultiplyO(matrix, (f32[]){ 0.5, -0.5, -0.5}, corners[4]);
    VM44_V3MultiplyO(matrix, (f32[]){ 0.5, -0.5,  0.5}, corners[5]);
    VM44_V3MultiplyO(matrix, (f32[]){ 0.5,  0.5, -0.5}, corners[6]);
    VM44_V3MultiplyO(matrix, (f32[]){ 0.5,  0.5,  0.5}, corners[7]);
}

void iVSHAPE_AxisProjectPoint(f32 axis[3], f32 point[3], f32 to[3]) {
    VM3_ScaleO(axis, VM3_Dot(axis, point), to);
}

bool iVSHAPE_CornersAxisSAT(f32 cornersA[8][3], f32 cornersB[8][3], f32 axis[3]) {
    if (VM3_Length(axis) == 0.0f) return false;
    f32 min_a_val, min_b_val, max_a_val, max_b_val;
    min_a_val = min_b_val = FLT_MAX;
    max_a_val = max_b_val = FLT_MIN;
    
    for (u32 i = 0; i < 8; i++) {
	f32 a_val = VM3_Dot(cornersA[i], axis);
	min_a_val = MIN(min_a_val, a_val);
	max_a_val = MAX(max_a_val, a_val);
	
	f32 b_val = VM3_Dot(cornersB[i], axis);
	min_b_val = MIN(min_b_val, b_val);
	max_b_val = MAX(max_b_val, b_val);
    }

    f32 long_span = MAX(max_a_val, max_b_val) - MIN(min_a_val, min_b_val);
    f32 sum_span = max_a_val - min_a_val + max_b_val - min_b_val;
    return long_span >= sum_span; // > to treat touching as intersection
}

bool iVSHAPE_CollisionBoxesGet(VShape a, VShape b) {
    // 3d cube Separating Axis Theorem
    f32 cornersA[8][3];
    f32 cornersB[8][3];
    iVSHAPE_BoxCornersGet(a, cornersA);
    iVSHAPE_BoxCornersGet(b, cornersB);

    f32 axis[3], first[3], second[3];
    f32 a_edge0[3], a_edge1[3], a_edge2[3];
    f32 b_edge0[3], b_edge1[3], b_edge2[3];
    VM3_SubtractO(cornersA[1], cornersA[0], a_edge0);
    VM3_SubtractO(cornersA[2], cornersA[0], a_edge1);
    VM3_SubtractO(cornersA[4], cornersA[0], a_edge2);
    
    VM3_SubtractO(cornersB[1], cornersB[0], b_edge0);
    VM3_SubtractO(cornersB[2], cornersB[0], b_edge1);
    VM3_SubtractO(cornersB[4], cornersB[0], b_edge2);    
    
    bool result = false;
    // 3 face perpendiculars (not normalized)

    // Box A
    VM3_CrossO(a_edge0, a_edge1, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;

    VM3_CrossO(a_edge0, a_edge2, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;

    VM3_CrossO(a_edge1, a_edge2, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;

    // Box B
    VM3_CrossO(b_edge0, b_edge1, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;

    VM3_CrossO(b_edge0, b_edge2, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;

    VM3_CrossO(b_edge1, b_edge2, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;

    
    // 9 mutual axies
    VM3_CrossO(a_edge0, b_edge0, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;

    VM3_CrossO(a_edge0, b_edge1, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;

    VM3_CrossO(a_edge0, b_edge2, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;


    VM3_CrossO(a_edge1, b_edge0, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;

    VM3_CrossO(a_edge1, b_edge1, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;

    VM3_CrossO(a_edge1, b_edge2, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;

    
    VM3_CrossO(a_edge2, b_edge0, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;

    VM3_CrossO(a_edge2, b_edge1, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;

    VM3_CrossO(a_edge2, b_edge2, axis);
    result = iVSHAPE_CornersAxisSAT(cornersA, cornersB, axis);
    if (result) return false;

    
    return true;
}


// AABB broad phase
