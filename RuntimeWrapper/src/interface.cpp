#include "interface.h"
#include "NavMeshWrapper.h"

using namespace nd;

void* nd_makeNavmeshWrapper(const char* binFilePath) {
	NavMeshWrapper* navmesh = new NavMeshWrapper();
	if (!navmesh->init(binFilePath)) {
		delete navmesh;
		return nullptr;
	}
	return navmesh;
}

void nd_freeNavmeshWrapper(void**p) {
	NavMeshWrapper* navmesh = (NavMeshWrapper*)*p;
	delete navmesh;
	*p = nullptr;
}

float nd_getAreaCost(void*p, int area){
	if (p == nullptr) { return false; }
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;

	return navmesh->getAreaCost(area);
}

bool nd_setAreaCost(void*p, int area, float cost){
	if (p == nullptr) { return false; }
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;

	return navmesh->setAreaCost(area, cost);
}

unsigned short nd_getIncludeFlag(void*p){
	if (p == nullptr) { return false; }
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;

	return navmesh->getIncludeFlag();
}

void nd_setIncludeFlag(void*p, unsigned short flag){
	if (p == nullptr) { return ; }
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;

	navmesh->setIncludeFlag(flag);
}

unsigned short nd_getExcludeFlag(void*p){
	if (p == nullptr) { return false; }
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;

	return navmesh->getExcludeFlag();
}

void nd_setExcludeFlag(void*p, unsigned short flag){
	if (p == nullptr) { return ; }
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;

	navmesh->setExcludeFlag(flag);
}

void nd_getQueryExt(void*p, float* extVector3){
	if (p == nullptr) { return; }
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;

	navmesh->getQueryExt(extVector3);
}

void nd_setQueryExt(void*p, float* extVector3){
	if (p == nullptr) { return ; }
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;

	navmesh->setQueryExt(extVector3);
}

bool nd_findPath(void* p, float* start, float* end, float* pathArray, int* floatNum) {
	if (p == nullptr) { return false; }
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	std::vector<float> path;
	if (!navmesh->findPath(start, end, path)) {
		return false;
	}

	memset(pathArray, 0, *floatNum * sizeof(float));
	int arraySize = *floatNum > path.size() ? (int)path.size() : *floatNum;
	for (size_t i = 0; i < arraySize; i++) {
		pathArray[i] = path[i];
	}
	*floatNum = arraySize;
	return true;
}

bool nd_findFullDirectPath(void* p, float* start, float* end, float* pathArray, int* floatNum, int testAdvanceSteps) {
	if (p == nullptr) { return false; }
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	std::vector<float> path;
	unsigned maxPoints = (unsigned)(*floatNum) / 3;
	if (!navmesh->findFullDirectPath(start, end, path, maxPoints, testAdvanceSteps)) {
		return false;
	}

	memset(pathArray, 0, *floatNum * sizeof(float));
	int arraySize = *floatNum > path.size() ? (int)path.size() : *floatNum;
	for (size_t i = 0; i < arraySize; i++) {
		pathArray[i] = path[i];
	}
	*floatNum = arraySize;
	return true;
}


bool nd_closestPointOnMesh(void* p, float* testPoint, float* inMeshPoint) {
	if (p == nullptr) { return false; }
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->closestPointOnMesh(testPoint, inMeshPoint);
}

float nd_raycast(void* p, float* start, float* end) {
	if (p == nullptr) { return -1; }
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->raycast(start, end);
}


//-----------------------------------------------------------------------------
//export raw query
dtStatus ndraw_findPath(void* p, dtPolyRef startRef, dtPolyRef endRef, const float* startPos, const float* endPos, dtPolyRef* path, int* pathCount, const int maxPath) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->findPath(startRef, endRef, startPos, endPos, path, pathCount, maxPath);
}

dtStatus ndraw_findStraightPath(void* p, const float* startPos, const float* endPos, const dtPolyRef* path, const int pathSize, float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs, int* straightPathCount, const int maxStraightPath, const int options) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->findStraightPath(startPos, endPos, path, pathSize, straightPath, straightPathFlags, straightPathRefs, straightPathCount, maxStraightPath, options);
}

dtStatus ndraw_initSlicedFindPath(void* p, dtPolyRef startRef, dtPolyRef endRef, const float* startPos, const float* endPos, const unsigned int options) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->initSlicedFindPath(startRef, endRef, startPos, endPos, options);
}

dtStatus ndraw_updateSlicedFindPath(void* p, const int maxIter, int* doneIters) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->updateSlicedFindPath(maxIter, doneIters);
}

dtStatus ndraw_finalizeSlicedFindPath(void* p, dtPolyRef* path, int* pathCount, const int maxPath) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->finalizeSlicedFindPath(path, pathCount, maxPath);
}

dtStatus ndraw_finalizeSlicedFindPathPartial(void* p, const dtPolyRef* existing, const int existingSize, dtPolyRef* path, int* pathCount, const int maxPath) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->finalizeSlicedFindPathPartial(existing, existingSize, path, pathCount, maxPath);
}

dtStatus ndraw_findPolysAroundCircle(void* p, dtPolyRef startRef, const float* centerPos, const float radius, dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost, int* resultCount, const int maxResult) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->findPolysAroundCircle(startRef, centerPos, radius, resultRef, resultParent, resultCost, resultCount, maxResult);
}

dtStatus ndraw_findPolysAroundShape(void* p, dtPolyRef startRef, const float* verts, const int nverts, dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost, int* resultCount, const int maxResult) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->findPolysAroundShape(startRef, verts, nverts, resultRef, resultParent, resultCost, resultCount, maxResult);
}

dtStatus ndraw_getPathFromDijkstraSearch(void* p, dtPolyRef endRef, dtPolyRef* path, int* pathCount, int maxPath) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->getPathFromDijkstraSearch(endRef, path, pathCount, maxPath);
}

dtStatus ndraw_findNearestPoly(void* p, const float* center, const float* halfExtents, dtPolyRef* nearestRef, float* nearestPt) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->findNearestPoly(center, halfExtents, nearestRef, nearestPt);
}

dtStatus ndraw_queryPolygons(void* p, const float* center, const float* halfExtents, dtPolyRef* polys, int* polyCount, const int maxPolys) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->queryPolygons(center, halfExtents, polys, polyCount, maxPolys);
}

dtStatus ndraw_findLocalNeighbourhood(void* p, dtPolyRef startRef, const float* centerPos, const float radius, dtPolyRef* resultRef, dtPolyRef* resultParent, int* resultCount, const int maxResult) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->findLocalNeighbourhood(startRef, centerPos, radius, resultRef, resultParent, resultCount, maxResult);
}

dtStatus ndraw_moveAlongSurface(void* p, dtPolyRef startRef, const float* startPos, const float* endPos, float* resultPos, dtPolyRef* visited, int* visitedCount, const int maxVisitedSize) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->moveAlongSurface(startRef, startPos, endPos, resultPos, visited, visitedCount, maxVisitedSize);
}

dtStatus ndraw_raycast(void* p, dtPolyRef startRef, const float* startPos, const float* endPos, float* t, float* hitNormal, dtPolyRef* path, int* pathCount, const int maxPath) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->raycast(startRef, startPos, endPos, t, hitNormal, path, pathCount, maxPath);
}

dtStatus ndraw_findDistanceToWall(void* p, dtPolyRef startRef, const float* centerPos, const float maxRadius, float* hitDist, float* hitPos, float* hitNormal) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->findDistanceToWall(startRef, centerPos, maxRadius, hitDist, hitPos, hitNormal);
}

dtStatus ndraw_getPolyWallSegments(void* p, dtPolyRef ref, float* segmentVerts, dtPolyRef* segmentRefs, int* segmentCount, const int maxSegments) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->getPolyWallSegments(ref, segmentVerts, segmentRefs, segmentCount, maxSegments);
}

dtStatus ndraw_findRandomPoint(void* p, dtPolyRef* randomRef, float* randomPt) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->findRandomPoint(randomRef, randomPt);
}

dtStatus ndraw_findRandomPointAroundCircle(void* p, dtPolyRef startRef, const float* centerPos, const float maxRadius, dtPolyRef* randomRef, float* randomPt) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->findRandomPointAroundCircle(startRef, centerPos, maxRadius, randomRef, randomPt);
}

dtStatus ndraw_closestPointOnPoly(void* p, dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->closestPointOnPoly(ref, pos, closest, posOverPoly);
}

dtStatus ndraw_closestPointOnPolyBoundary(void* p, dtPolyRef ref, const float* pos, float* closest) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->closestPointOnPolyBoundary(ref, pos, closest);
}

dtStatus ndraw_getPolyHeight(void* p, dtPolyRef ref, const float* pos, float* height) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->getPolyHeight(ref, pos, height);
}

bool ndraw_isValidPolyRef(void* p, dtPolyRef ref) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->isValidPolyRef(ref);
}

bool ndraw_isInClosedList(void* p, dtPolyRef ref) {
    if (p == nullptr){return DT_INVALID_PARAM;}
	NavMeshWrapper* navmesh = (NavMeshWrapper*)p;
	return navmesh->isInClosedList(ref);
}

