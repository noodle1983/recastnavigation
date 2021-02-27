#ifndef NAVMESHWRAPPER_H
#define NAVMESHWRAPPER_H

#include <string>
#include <vector>
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

namespace nd{

    class NavMeshWrapper{
    public:
        static const int MAX_POLYS = 256;
        static const int MAX_SMOOTH = 2048;

        NavMeshWrapper();
        virtual ~NavMeshWrapper();

        bool init(const char* path);
        bool findPath(float* start, float *end, std::vector<float>& path);

    private:
        dtNavMesh* navMeshM;
        dtNavMeshQuery* navQueryM;

        dtQueryFilter queryFilterM;
        dtPolyRef queryPolysM[MAX_POLYS];
        float queryExtM[3];
    };
}

#endif /* NAVMESHWRAPPER_H */

