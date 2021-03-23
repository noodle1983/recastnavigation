#ifndef INTERFACE_H
#define INTERFACE_H

#if defined (_MSC_VER)
#define LIB_API __declspec(dllexport)
#else
#define LIB_API
#endif

typedef unsigned int dtStatus;
#ifdef DT_POLYREF64
static const unsigned int DT_SALT_BITS = 16;
static const unsigned int DT_TILE_BITS = 28;
static const unsigned int DT_POLY_BITS = 20;
typedef uint64_t dtPolyRef;
#else
typedef unsigned int dtPolyRef;
#endif

extern "C" {
	LIB_API void* nd_makeNavmeshWrapper(const char* binFilePath);
	LIB_API void nd_freeNavmeshWrapper(void**);

	LIB_API	float nd_getAreaCost(void*p, int area);
	LIB_API	bool nd_setAreaCost(void*p, int area, float cost);
	LIB_API	unsigned short nd_getIncludeFlag(void*p);
	LIB_API	void nd_setIncludeFlag(void*p, unsigned short flag);
	LIB_API	unsigned short nd_getExcludeFlag(void*p);
	LIB_API	void nd_setExcludeFlag(void*p, unsigned short flag);
	LIB_API	void nd_getQueryExt(void*p, float* extVector3);
	LIB_API	void nd_setQueryExt(void*p, float* extVector3);

	LIB_API bool nd_closestPointOnMesh(void* p, float* testPoint, float *inMeshPoint);
	LIB_API bool nd_findPath(void* p, float* start, float *end, float* pathArray, int* len);
	LIB_API bool nd_findFullDirectPath(void* p, float* start, float *end, float* pathArray, int* len, int testAdvanceSteps);
	LIB_API float nd_raycast(void* p, float* start, float *end);

	//export raw query
	LIB_API dtStatus ndraw_findPath(void* p, dtPolyRef startRef, dtPolyRef endRef, const float* startPos, const float* endPos, dtPolyRef* path, int* pathCount, const int maxPath) ;
	LIB_API dtStatus ndraw_findStraightPath(void* p, const float* startPos, const float* endPos, const dtPolyRef* path, const int pathSize, float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs, int* straightPathCount, const int maxStraightPath, const int options = 0) ;
	LIB_API dtStatus ndraw_initSlicedFindPath(void* p, dtPolyRef startRef, dtPolyRef endRef, const float* startPos, const float* endPos, const unsigned int options = 0) ;
	LIB_API dtStatus ndraw_updateSlicedFindPath(void* p, const int maxIter, int* doneIters) ;
	LIB_API dtStatus ndraw_finalizeSlicedFindPath(void* p, dtPolyRef* path, int* pathCount, const int maxPath) ;
	LIB_API dtStatus ndraw_finalizeSlicedFindPathPartial(void* p, const dtPolyRef* existing, const int existingSize, dtPolyRef* path, int* pathCount, const int maxPath) ;
	LIB_API dtStatus ndraw_findPolysAroundCircle(void* p, dtPolyRef startRef, const float* centerPos, const float radius, dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost, int* resultCount, const int maxResult) ;
	LIB_API dtStatus ndraw_findPolysAroundShape(void* p, dtPolyRef startRef, const float* verts, const int nverts, dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost, int* resultCount, const int maxResult) ;
	LIB_API dtStatus ndraw_getPathFromDijkstraSearch(void* p, dtPolyRef endRef, dtPolyRef* path, int* pathCount, int maxPath) ;
	LIB_API dtStatus ndraw_findNearestPoly(void* p, const float* center, const float* halfExtents, dtPolyRef* nearestRef, float* nearestPt) ;
	LIB_API dtStatus ndraw_queryPolygons(void* p, const float* center, const float* halfExtents, dtPolyRef* polys, int* polyCount, const int maxPolys) ;
	LIB_API dtStatus ndraw_findLocalNeighbourhood(void* p, dtPolyRef startRef, const float* centerPos, const float radius, dtPolyRef* resultRef, dtPolyRef* resultParent, int* resultCount, const int maxResult) ;
	LIB_API dtStatus ndraw_moveAlongSurface(void* p, dtPolyRef startRef, const float* startPos, const float* endPos, float* resultPos, dtPolyRef* visited, int* visitedCount, const int maxVisitedSize) ;
	LIB_API dtStatus ndraw_raycast(void* p, dtPolyRef startRef, const float* startPos, const float* endPos, float* t, float* hitNormal, dtPolyRef* path, int* pathCount, const int maxPath) ;
	LIB_API dtStatus ndraw_findDistanceToWall(void* p, dtPolyRef startRef, const float* centerPos, const float maxRadius, float* hitDist, float* hitPos, float* hitNormal) ;
	LIB_API dtStatus ndraw_getPolyWallSegments(void* p, dtPolyRef ref, float* segmentVerts, dtPolyRef* segmentRefs, int* segmentCount, const int maxSegments) ;
	LIB_API dtStatus ndraw_findRandomPoint(void* p, dtPolyRef* randomRef, float* randomPt) ;
	LIB_API dtStatus ndraw_findRandomPointAroundCircle(void* p, dtPolyRef startRef, const float* centerPos, const float maxRadius, dtPolyRef* randomRef, float* randomPt) ;
	LIB_API dtStatus ndraw_closestPointOnPoly(void* p, dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) ;
	LIB_API dtStatus ndraw_closestPointOnPolyBoundary(void* p, dtPolyRef ref, const float* pos, float* closest) ;
	LIB_API dtStatus ndraw_getPolyHeight(void* p, dtPolyRef ref, const float* pos, float* height) ;
	LIB_API bool ndraw_isValidPolyRef(void* p, dtPolyRef ref) ;
	LIB_API bool ndraw_isInClosedList(void* p, dtPolyRef ref) ;
};

#endif
