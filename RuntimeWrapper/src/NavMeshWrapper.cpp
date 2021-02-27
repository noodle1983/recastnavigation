#include "NavMeshWrapper.h"
#include "DetourCommon.h"
#include <fstream>
#include <cstring>
#include <vector>

using namespace nd;
using namespace std;

//-----------------------------------
// keep the same with sample.cpp
//-----------------------------------
static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

enum SamplePolyFlags
{
    SAMPLE_POLYFLAGS_WALK = 0x01,       // Ability to walk (ground, grass, road)
    SAMPLE_POLYFLAGS_SWIM = 0x02,       // Ability to swim (water).
    SAMPLE_POLYFLAGS_DOOR = 0x04,       // Ability to move through doors.
    SAMPLE_POLYFLAGS_JUMP = 0x08,       // Ability to jump.
    SAMPLE_POLYFLAGS_DISABLED = 0x10,       // Disabled polygon
    SAMPLE_POLYFLAGS_ALL = 0xffff   // All abilities.
};

dtNavMesh* loadAll(const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp) return 0;

	// Read header.
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (readLen != 1)
	{
		fclose(fp);
		return 0;
	}
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return 0;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		fclose(fp);
		return 0;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return 0;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (readLen != 1)
		{
			fclose(fp);
			return 0;
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		readLen = fread(data, tileHeader.dataSize, 1, fp);
		if (readLen != 1)
		{
			dtFree(data);
			fclose(fp);
			return 0;
		}

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	fclose(fp);

	return mesh;
}

//-----------------------------------

NavMeshWrapper::NavMeshWrapper()
    : navMeshM(nullptr)
    , navQueryM(nullptr)
    , queryExtM{1, 1, 1}
{
    queryFilterM.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
    queryFilterM.setExcludeFlags(0);
}

NavMeshWrapper::~NavMeshWrapper(){
    if (navMeshM) { dtFreeNavMesh(navMeshM); }
    if (navQueryM) { dtFreeNavMeshQuery(navQueryM); }
}

bool NavMeshWrapper::init(const char* path){
	if(navMeshM != nullptr){ dtFreeNavMesh(navMeshM); }
    navMeshM = loadAll(path);
    if (navMeshM == nullptr) {return false;}

    navQueryM->init(navMeshM, 2048);
    return true;
}

bool NavMeshWrapper::findPath(float* start, float *end, vector<float>& path) {
    dtPolyRef startRef = 0;

    navQueryM->findNearestPoly(start, queryExtM, &queryFilterM, &startRef, 0);
    if (startRef == 0) {
        return false;
    }

    dtPolyRef endRef = 0;
    navQueryM->findNearestPoly(end, queryExtM, &queryFilterM, &endRef, 0);
    if (endRef == 0) {
        return false;
    }

    int numPolys = 0;
    navQueryM->findPath(startRef, endRef, start, end, &queryFilterM, queryPolysM, &numPolys, MAX_POLYS);
    if (numPolys == 0) {
        return false;
    }

    float epos[3];
    dtVcopy(epos, end);
    if (queryPolysM[numPolys - 1] != endRef) {
        navQueryM->closestPointOnPoly(queryPolysM[numPolys - 1], end, epos, 0);
    }

    float detourPath[MAX_POLYS * 3];
    int pathNum;
    navQueryM->findStraightPath(start, epos, queryPolysM, numPolys,
        detourPath, nullptr, nullptr, &pathNum, MAX_POLYS, 0);

    path.clear();
    for (int i = pathNum - 1; i >= 0; --i) {
        path.emplace_back(detourPath[i * 3]);
        path.emplace_back(detourPath[i * 3 + 1]);
        path.emplace_back(detourPath[i * 3 + 2]);
    }
    return true;
}


