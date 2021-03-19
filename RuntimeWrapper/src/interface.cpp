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
