#ifndef NAVMESHWRAPPER_H
#define NAVMESHWRAPPER_H

#include <string>
#include <vector>
#include "nd_header.h"

namespace nd{
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

    class NavMeshWrapper{
    public:
        static const int MAX_POLYS = 2048;
        static const int MAX_SMOOTH = 2048;

        NavMeshWrapper();
        virtual ~NavMeshWrapper();

        bool init(const char* path);

		float getAreaCost(int area);
		bool setAreaCost(int area, float cost);
		unsigned short getIncludeFlag();
		void setIncludeFlag(unsigned short flag);
		unsigned short getExcludeFlag();
		void setExcludeFlag(unsigned short flag);
		void getQueryExt(float* extVector3);
		void setQueryExt(float* extVector3);
		
        float raycast(float* start, float *end);
        bool findPath(float* start, float *end, std::vector<float>& path);
        bool findFullDirectPath(float* start, float *end, std::vector<float>& path, unsigned maxPoints = 512, unsigned testAdvanceSteps = 1);
		bool closestPointOnMesh(float* testPoint, float* inMeshPoint);

		//export raw query
		dtStatus findPath(dtPolyRef startRef, dtPolyRef endRef,
			const float* startPos, const float* endPos,
			dtPolyRef* path, int* pathCount, const int maxPath) {
			return navQueryM->findPath(startRef, endRef, startPos, endPos, &queryFilterM, path, pathCount, maxPath);
		}
		dtStatus findStraightPath(const float* startPos, const float* endPos,
			const dtPolyRef* path, const int pathSize,
			float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
			int* straightPathCount, const int maxStraightPath, const int options = 0) {
			return navQueryM->findStraightPath(startPos, endPos, path, pathSize, straightPath, straightPathFlags, straightPathRefs, straightPathCount, maxStraightPath, options);
		}
		dtStatus initSlicedFindPath(dtPolyRef startRef, dtPolyRef endRef,
			const float* startPos, const float* endPos, const unsigned int options = 0) {
			return navQueryM->initSlicedFindPath(startRef, endRef, startPos, endPos, &queryFilterM, options);
		}
		dtStatus updateSlicedFindPath(const int maxIter, int* doneIters) {
			return navQueryM->updateSlicedFindPath(maxIter, doneIters);
		}
		dtStatus finalizeSlicedFindPath(dtPolyRef* path, int* pathCount, const int maxPath) {
			return navQueryM->finalizeSlicedFindPath(path, pathCount, maxPath);
		}
		dtStatus finalizeSlicedFindPathPartial(const dtPolyRef* existing, const int existingSize,
			dtPolyRef* path, int* pathCount, const int maxPath) {
			return navQueryM->finalizeSlicedFindPathPartial(existing, existingSize, path, pathCount, maxPath);
		}
		dtStatus findPolysAroundCircle(dtPolyRef startRef, const float* centerPos, const float radius,
			dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
			int* resultCount, const int maxResult) {
			return navQueryM->findPolysAroundCircle(startRef, centerPos, radius, &queryFilterM, resultRef, resultParent, resultCost, resultCount, maxResult);
		}
		dtStatus findPolysAroundShape(dtPolyRef startRef, const float* verts, const int nverts,
			dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
			int* resultCount, const int maxResult) {
			return navQueryM->findPolysAroundShape(startRef, verts, nverts, &queryFilterM, resultRef, resultParent, resultCost, resultCount, maxResult);
		}
		dtStatus getPathFromDijkstraSearch(dtPolyRef endRef, dtPolyRef* path, int* pathCount, int maxPath) {
			return navQueryM->getPathFromDijkstraSearch(endRef, path, pathCount, maxPath);
		}
		dtStatus findNearestPoly(const float* center, const float* halfExtents,
			dtPolyRef* nearestRef, float* nearestPt) {
			return navQueryM->findNearestPoly(center, halfExtents, &queryFilterM, nearestRef, nearestPt);
		}
		dtStatus queryPolygons(const float* center, const float* halfExtents,
			dtPolyRef* polys, int* polyCount, const int maxPolys) {
			return navQueryM->queryPolygons(center, halfExtents, &queryFilterM, polys, polyCount, maxPolys);
		}
		dtStatus findLocalNeighbourhood(dtPolyRef startRef, const float* centerPos, const float radius,
			dtPolyRef* resultRef, dtPolyRef* resultParent,
			int* resultCount, const int maxResult) {
			return navQueryM->findLocalNeighbourhood(startRef, centerPos, radius, &queryFilterM, resultRef, resultParent, resultCount, maxResult);
		}
		dtStatus moveAlongSurface(dtPolyRef startRef, const float* startPos, const float* endPos,
			float* resultPos, dtPolyRef* visited, int* visitedCount, const int maxVisitedSize) {
			return navQueryM->moveAlongSurface(startRef, startPos, endPos, &queryFilterM, resultPos, visited, visitedCount, maxVisitedSize);
		}
		dtStatus raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
			float* t, float* hitNormal, dtPolyRef* path, int* pathCount, const int maxPath) {
			return navQueryM->raycast(startRef, startPos, endPos, &queryFilterM, t, hitNormal, path, pathCount, maxPath);
		}
		dtStatus findDistanceToWall(dtPolyRef startRef, const float* centerPos, const float maxRadius,
			float* hitDist, float* hitPos, float* hitNormal) {
			return navQueryM->findDistanceToWall(startRef, centerPos, maxRadius, &queryFilterM, hitDist, hitPos, hitNormal);
		}
		dtStatus getPolyWallSegments(dtPolyRef ref,
			float* segmentVerts, dtPolyRef* segmentRefs, int* segmentCount,
			const int maxSegments) {
			return navQueryM->getPolyWallSegments(ref, &queryFilterM, segmentVerts, segmentRefs, segmentCount, maxSegments);
		}
		static float frand() { return static_cast <float> (rand()) / static_cast <float> (RAND_MAX); }
		dtStatus findRandomPoint(dtPolyRef* randomRef, float* randomPt) {
			return navQueryM->findRandomPoint(&queryFilterM, frand, randomRef, randomPt);
		}
		dtStatus findRandomPointAroundCircle(dtPolyRef startRef, const float* centerPos, const float maxRadius, dtPolyRef* randomRef, float* randomPt) {
			return navQueryM->findRandomPointAroundCircle(startRef, centerPos, maxRadius, &queryFilterM, frand, randomRef, randomPt);
		}
		dtStatus closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) {
			return navQueryM->closestPointOnPoly(ref, pos, closest, posOverPoly);
		}
		dtStatus closestPointOnPolyBoundary(dtPolyRef ref, const float* pos, float* closest) {
			return navQueryM->closestPointOnPolyBoundary(ref, pos, closest);
		}
		dtStatus getPolyHeight(dtPolyRef ref, const float* pos, float* height) {
			return navQueryM->getPolyHeight(ref, pos, height);
		}
		bool isValidPolyRef(dtPolyRef ref) {
			return navQueryM->isValidPolyRef(ref, &queryFilterM);
		}
		bool isInClosedList(dtPolyRef ref) {
			return navQueryM->isInClosedList(ref);
		}
	

    private:
        dtNavMesh* navMeshM;
        dtNavMeshQuery* navQueryM;

        dtQueryFilter queryFilterM;
        dtPolyRef queryPolysM[MAX_POLYS];
        float queryExtM[3];
    };
}

#endif /* NAVMESHWRAPPER_H */

