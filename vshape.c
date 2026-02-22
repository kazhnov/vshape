#include "vshape.h"
#include "include/vmath/vmath.h"
#include <assert.h>
#include <float.h>
#include <string.h>
#include <math.h>

#define MAX(a, b) (a > b ? a : b)
#define MIN(a, b) (a < b ? a : b)

bool iVSHAPE_CollisionSpheresGet(VShape a, VShape b);
bool iVSHAPE_CollisionBoxesGet(VShape a, VShape b);
bool iVSHAPE_CollisionAABBsGet(VShape a, VShape b);
bool iVSHAPE_AABBsCollisionGet(VShape a, VShape b, f32 to_separate[3]);
bool iVSHAPE_AABBsCollide(VShape a, VShape b);
bool iVSHAPE_GJK(u32 vcount1, f32 verts1[vcount1][3], u32 vcount2, f32 verts2[vcount2][3]);
bool iVSHAPE_CollisionFrustrumToBoxGet(VShape a, VShape b);

void iVSHAPE_BoxCornersGet(VShape box, float corners[8][3]);
void iVSHAPE_FrustrumCornersGet(VShape frustrum, float corners[8][3]);

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

VShape VSHAPE_FrustrumCreate(float center[3], float radius, float fov, float ratio) {
    VShape frustrum = {0};
    frustrum.type = VSHAPE_FRUSTRUM;
    VM3_Copy(frustrum.center, center);
    frustrum.size[0] = fov;
    frustrum.size[1] = fov/ratio;
    frustrum.size[2] = radius;
    return frustrum;
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


bool VSHAPE_CollisionGet(VShape a, VShape b, f32 to_separate[3]) {
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
	{
	    f32 cornersA[8][3]; f32 cornersB[8][3];
	    iVSHAPE_BoxCornersGet(a, cornersA);
	    iVSHAPE_BoxCornersGet(b, cornersB);
	    return iVSHAPE_GJK(8, cornersA, 8, cornersB);
	}
	case VSHAPE_FRUSTRUM:
	{
	    f32 boxCorners[8][3]; f32 frustrumCorners[5][3];
	    iVSHAPE_BoxCornersGet(a, boxCorners);
	    iVSHAPE_FrustrumCornersGet(b, frustrumCorners);
	    return iVSHAPE_GJK(8, boxCorners, 5, frustrumCorners);
	}
	default:
	    break;
	}
	break;
    case VSHAPE_FRUSTRUM:
	switch (b.type) {
	case VSHAPE_BOX:
	{
	    f32 boxCorners[8][3]; f32 frustrumCorners[5][3];
	    iVSHAPE_BoxCornersGet(b, boxCorners);
	    iVSHAPE_FrustrumCornersGet(a, frustrumCorners);
	    return iVSHAPE_GJK(8, boxCorners, 5, frustrumCorners);
	}
	default:
	    assert(false);
	    break;
	}
    case VSHAPE_AABB:
	switch (b.type) {
	case VSHAPE_SPHERE:
	    break;
	case VSHAPE_BOX:
	    break;
	case VSHAPE_AABB:
	    return iVSHAPE_AABBsCollisionGet(a, b, to_separate);
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

bool VSHAPE_Collide(VShape a, VShape b) {
    float to_separate[3];
    return VSHAPE_CollisionGet(a, b, to_separate);
}


bool iVSHAPE_CollisionSpheresGet(VShape a, VShape b) {
    return VM3_Distance(a.center, b.center) < a.radius + b.radius;
}


bool iVSHAPE_AABBsCollisionGet(VShape a, VShape b, f32 to_separate[3]) {
    f32 amin[3]; f32 amax[3];
    amin[0] = a.center[0] - a.size[0]/2.f;
    amin[1] = a.center[1] - a.size[1]/2.f;
    amin[2] = a.center[2] - a.size[2]/2.f;
    amax[0] = a.center[0] + a.size[0]/2.f;
    amax[1] = a.center[1] + a.size[1]/2.f;
    amax[2] = a.center[2] + a.size[2]/2.f;

    f32 bmin[3]; f32 bmax[3];
    bmin[0] = b.center[0] - b.size[0]/2.f;
    bmin[1] = b.center[1] - b.size[1]/2.f;
    bmin[2] = b.center[2] - b.size[2]/2.f;
    bmax[0] = b.center[0] + b.size[0]/2.f;
    bmax[1] = b.center[1] + b.size[1]/2.f;
    bmax[2] = b.center[2] + b.size[2]/2.f;


    if (amin[0] < bmax[0] && bmin[0] < amax[0] &&
	amin[1] < bmax[1] && bmin[1] < amax[1] &&
	amin[2] < bmax[2] && bmin[2] < amax[2]
	) {
	VM3_Set(to_separate, 0, 0, 0);
	// Check a to b
	to_separate[0] = amin[0] > bmax[0] ? to_separate[0] : bmax[0] - amin[0];
	to_separate[1] = amin[1] > bmax[1] ? to_separate[1] : bmax[1] - amin[1];
	to_separate[2] = amin[2] > bmax[2] ? to_separate[2] : bmax[2] - amin[2];    

	// Check b to a
	to_separate[0] = bmin[0] > amax[0] ? to_separate[0] : amax[0] - bmin[0];
	to_separate[1] = bmin[1] > amax[1] ? to_separate[1] : amax[1] - bmin[1];
	to_separate[2] = bmin[2] > amax[2] ? to_separate[2] : amax[2] - bmin[2];
	return true;
    } else {
	return false;
    }
	
}

bool iVSHAPE_AABBsCollide(VShape a, VShape b) {
    f32 to_separate[3];
    return iVSHAPE_AABBsCollisionGet(a, b, to_separate);
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

void iVSHAPE_FrustrumCornersGet(VShape frustrum, float corners[5][3]) {
    float matrix[16] = VM44_IDENTITY;
    VM44_Rotate(matrix, frustrum.rotation);
    float radius = frustrum.size[2];
    VM44_Scale(matrix, (f32[]){radius, radius, radius});
    VM44_Translate(matrix, frustrum.center);


    float width = tanf(frustrum.size[0]/2);
    float height = tanf(frustrum.size[1]/2);
    
    VM44_V3MultiplyO(matrix, (f32[]){ 0.0,  0.0,  0.0}, corners[0]);
    VM44_V3MultiplyO(matrix, (f32[]){-width, -height, -1.0}, corners[1]);
    VM44_V3MultiplyO(matrix, (f32[]){-width,  height, -1.0}, corners[2]);
    VM44_V3MultiplyO(matrix, (f32[]){ width, -height, -1.0}, corners[3]);
    VM44_V3MultiplyO(matrix, (f32[]){ width,  height, -1.0}, corners[4]);
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
    return long_span > sum_span; // > to treat touching as intersection
}

bool iVSHAPE_CollisionBoxesGet(VShape a, VShape b) {
    // 3d cube Separating Axis Theorem
    f32 cornersA[8][3];
    f32 cornersB[8][3];
    iVSHAPE_BoxCornersGet(a, cornersA);
    iVSHAPE_BoxCornersGet(b, cornersB);

#if 1
    return iVSHAPE_GJK(8, cornersA, 8, cornersB);
#else 
    
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
#endif
}


// GJK
void iVSHAPE_GJKFurthestPoint(f32 (*verts)[3], u32 vcount, f32 axis[3], f32 out[3]) {
    f32 distance_max = -FLT_MAX;
    f32 distance;
    
    for (u32 i = 0; i < vcount; i++) {
	distance = VM3_Dot(verts[i], axis);
	if (distance > distance_max) {
	    VM3_Copy(out, verts[i]);
	    distance_max = distance;
	}
    }
}

void iVSHAPE_GJKSupport(f32 (*verts1)[3], u32 vcount1, f32 (*verts2)[3], u32 vcount2, f32 axis[3], f32 out[3]) {
    f32 first[3]; f32 second[3];
    f32 naxis[3];
    VM3_NegateO(axis, naxis);
    iVSHAPE_GJKFurthestPoint(verts1, vcount1,  axis, first);
    iVSHAPE_GJKFurthestPoint(verts2, vcount2, naxis, second);
    VM3_SubtractO(first, second, out);
}

typedef struct {
    f32 points[4][3];
    u32 size;
} GJKSimplex;

void iVSHAPE_GJKSimplexCopy(GJKSimplex *to, GJKSimplex *from) {
    memcpy(to, from, sizeof(GJKSimplex));
}

void iVSHAPE_GJKSimplexPush(GJKSimplex *to, f32 point[3]) {
    VM3_Copy(to->points[3], to->points[2]);
    VM3_Copy(to->points[2], to->points[1]);
    VM3_Copy(to->points[1], to->points[0]);
    VM3_Copy(to->points[0], point);
    to->size = MIN(to->size+1, 4);
}

void iVSHAPE_GJKSimplexClear(GJKSimplex *s) {
    memset(s, 0, sizeof(GJKSimplex));
}


bool iVSHAPE_GJKLine(GJKSimplex *points, f32 direction[3]) {
    f32 a[3]; f32 b[3]; f32 ab[3]; f32 ao[3]; f32 temp[3];
    VM3_Copy(a, points->points[0]);
    VM3_Copy(b, points->points[1]);

    VM3_SubtractO(b, a, ab);
    VM3_NegateO(     a, ao);

    if (VM3_SameDirection(ab, ao)) {
	VM3_CrossO(ab, ao, temp);
	VM3_CrossO(temp, ab, direction);
    } else {
	iVSHAPE_GJKSimplexClear(points);
	iVSHAPE_GJKSimplexPush(points, a);
	VM3_Copy(direction, ao);
    }

    return false;
}

bool iVSHAPE_GJKTriangle(GJKSimplex *points, f32 direction[3]) {
    f32 a[3]; f32 b[3]; f32 c[3];
    f32 ab[3]; f32 ac[3]; f32 ao[3];
    f32 abc[3]; f32 temp[3];

    VM3_Copy(a, points->points[0]);
    VM3_Copy(b, points->points[1]);
    VM3_Copy(c, points->points[2]);

    VM3_SubtractO(b, a, ab);    
    VM3_SubtractO(c, a, ac);
    VM3_NegateO(     a, ao);
    
    VM3_CrossO(ab, ac, abc);

    VM3_CrossO(abc, ac, temp);
    if (VM3_SameDirection(temp, ao)) {
	if (VM3_SameDirection(ac, ao)) {
	    iVSHAPE_GJKSimplexClear(points);
	    iVSHAPE_GJKSimplexPush(points, c);
	    iVSHAPE_GJKSimplexPush(points, a);
	    VM3_CrossO(ac, ao, temp);
	    VM3_CrossO(temp, ac, direction);
	} else {
	    iVSHAPE_GJKSimplexClear(points);	    
	    iVSHAPE_GJKSimplexPush(points, b);
	    iVSHAPE_GJKSimplexPush(points, a);

	    return iVSHAPE_GJKLine(points, direction);
	}
    } else {
	VM3_CrossO(ab, abc, temp);
	if (VM3_SameDirection(temp, ao)) {
	    iVSHAPE_GJKSimplexClear(points);
	    iVSHAPE_GJKSimplexPush(points, b);
	    iVSHAPE_GJKSimplexPush(points, a);

	    return iVSHAPE_GJKLine(points, direction);
	} else {
	    if (VM3_SameDirection(abc, ao)) {
		VM3_Copy(direction, abc);
	    } else {
		iVSHAPE_GJKSimplexClear(points);	    	    
		iVSHAPE_GJKSimplexPush(points, b);
		iVSHAPE_GJKSimplexPush(points, c);
		iVSHAPE_GJKSimplexPush(points, a);

		VM3_NegateO(abc, direction);
	    }
	}
    }
    
    return false;
}
bool iVSHAPE_GJKTetrahedron(GJKSimplex *points, f32 direction[3]) {
    f32 a[3]; f32 b[3]; f32 c[3]; f32 d[3];
    f32 ab[3]; f32 ac[3]; f32 ad[3]; f32 ao[3];
    f32 abc[3]; f32 acd[3]; f32 adb[3]; f32 temp[3];

    VM3_Copy(a, points->points[0]);
    VM3_Copy(b, points->points[1]);
    VM3_Copy(c, points->points[2]);
    VM3_Copy(d, points->points[3]);

    VM3_SubtractO(b, a, ab);
    VM3_SubtractO(c, a, ac);
    VM3_SubtractO(d, a, ad);
    VM3_NegateO(     a, ao);

    VM3_CrossO(ab, ac, abc);
    VM3_CrossO(ac, ad, acd);
    VM3_CrossO(ad, ab, adb);

    if (VM3_SameDirection(abc, ao)) {
	iVSHAPE_GJKSimplexClear(points);	    	    
	iVSHAPE_GJKSimplexPush(points, c);
	iVSHAPE_GJKSimplexPush(points, b);
	iVSHAPE_GJKSimplexPush(points, a);
	return iVSHAPE_GJKTriangle(points, direction);
    }
    if (VM3_SameDirection(acd, ao)) {
	iVSHAPE_GJKSimplexClear(points);	    	    
	iVSHAPE_GJKSimplexPush(points, d);
	iVSHAPE_GJKSimplexPush(points, c);
	iVSHAPE_GJKSimplexPush(points, a);	
	return iVSHAPE_GJKTriangle(points, direction);
    }
    if (VM3_SameDirection(adb, ao)) {	
	iVSHAPE_GJKSimplexClear(points);	    
	iVSHAPE_GJKSimplexPush(points, b);
	iVSHAPE_GJKSimplexPush(points, d);
	iVSHAPE_GJKSimplexPush(points, a);
	return iVSHAPE_GJKTriangle(points, direction);
    }
    
    return true;
}


bool iVSHAPE_GJKSimplexNext(GJKSimplex *points, f32 direction[3]) {
    
    switch (points->size) {
    case 2:
	return iVSHAPE_GJKLine(       points, direction);
    case 3:
	return iVSHAPE_GJKTriangle(   points, direction);
    case 4:
	return iVSHAPE_GJKTetrahedron(points, direction);
    }

    assert(false);
}


bool iVSHAPE_GJK(u32 vcount1, f32 verts1[vcount1][3], u32 vcount2,  f32 verts2[vcount2][3]) {
    // Get initial support point in any direction
    f32 support[3];
    iVSHAPE_GJKSupport(verts1, vcount1, verts2, vcount2, VM3_RIGHT, support);

    GJKSimplex points;
    iVSHAPE_GJKSimplexClear(&points);    
    iVSHAPE_GJKSimplexPush(&points, support);

    f32 direction[3];
    VM3_NegateO(support, direction);
    
    while (true) {
	iVSHAPE_GJKSupport(verts1, vcount1, verts2, vcount2, direction, support);

	if (!VM3_SameDirection(support, direction)) {
	    return false;
	}

	iVSHAPE_GJKSimplexPush(&points, support);

	if(iVSHAPE_GJKSimplexNext(&points, direction)) {
	    return true;
	}
    }
}
