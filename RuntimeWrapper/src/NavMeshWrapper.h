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
        static const int MAX_POLYS = 256;
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

    private:
        dtNavMesh* navMeshM;
        dtNavMeshQuery* navQueryM;

        dtQueryFilter queryFilterM;
        dtPolyRef queryPolysM[MAX_POLYS];
        float queryExtM[3];
    };
}

#endif /* NAVMESHWRAPPER_H */

