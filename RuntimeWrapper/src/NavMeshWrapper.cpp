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

    if (navQueryM) { dtFreeNavMeshQuery(navQueryM); }
    navQueryM = dtAllocNavMeshQuery();
    navQueryM->init(navMeshM, 2048);
    return true;
}

        dtQueryFilter queryFilterM;
        float queryExtM[3];
float NavMeshWrapper::getAreaCost(int area) {
	if (area < 0 || area >= DT_MAX_AREAS) { return -1; }
	return queryFilterM.getAreaCost(area);
}

bool NavMeshWrapper::setAreaCost(int area, float cost){
	if (area < 0 || area >= DT_MAX_AREAS) { return false; }
	queryFilterM.setAreaCost(area, cost);
	return true;
}

unsigned short NavMeshWrapper::getIncludeFlag(){
	return queryFilterM.getIncludeFlags();
}

void NavMeshWrapper::setIncludeFlag(unsigned short flag){
	return queryFilterM.setIncludeFlags(flag);
}

unsigned short NavMeshWrapper::getExcludeFlag() {
	return queryFilterM.getExcludeFlags();
}
void NavMeshWrapper::setExcludeFlag(unsigned short flag){
	return queryFilterM.setExcludeFlags(flag);
}

void NavMeshWrapper::getQueryExt(float* extVector3){
	dtVcopy(extVector3, queryExtM);
}

void NavMeshWrapper::setQueryExt(float* extVector3){
	dtVcopy(queryExtM, extVector3);
}

bool NavMeshWrapper::findPath(float* start, float *end, vector<float>& path) {
    dtPolyRef startRef = 0;

    dtStatus status = navQueryM->findNearestPoly(start, queryExtM, &queryFilterM, &startRef, 0);
	if (dtStatusFailed(status)){
		return false;
    }

    dtPolyRef endRef = 0;
    status = navQueryM->findNearestPoly(end, queryExtM, &queryFilterM, &endRef, 0);
	if (dtStatusFailed(status)){
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
    for (int i = 0; i < pathNum; i++) {
        path.emplace_back(detourPath[i * 3]);
        path.emplace_back(detourPath[i * 3 + 1]);
        path.emplace_back(detourPath[i * 3 + 2]);
    }
    return true;
}

bool NavMeshWrapper::findFullDirectPath(float* start, float* end, std::vector<float>& path, unsigned maxPoints, unsigned testAdvanceSteps) {
    dtPolyRef startRef = 0;

    dtStatus status = navQueryM->findNearestPoly(start, queryExtM, &queryFilterM, &startRef, 0);
	if (dtStatusFailed(status)){
		return false;
    }

    dtPolyRef endRef = 0;
    status = navQueryM->findNearestPoly(end, queryExtM, &queryFilterM, &endRef, 0);
	if (dtStatusFailed(status)){
		return false;
    }

	//full path
	std::vector<float> tmpPath;
	bool isCompleted = false;
	while (!isCompleted && tmpPath.size() < (size_t)maxPoints*3) {
		int numPolys = 0;
		navQueryM->findPath(startRef, endRef, start, end, &queryFilterM, queryPolysM, &numPolys, MAX_POLYS);
		if (numPolys == 0) {
			return false;
		}

		float epos[3];
		dtVcopy(epos, end);
		isCompleted = queryPolysM[numPolys - 1] == endRef;
		if (!isCompleted) {
			navQueryM->closestPointOnPoly(queryPolysM[numPolys - 1], end, epos, 0);
		}

		float detourPath[MAX_POLYS * 3];
		int pathNum;
		navQueryM->findStraightPath(start, epos, queryPolysM, numPolys,
			detourPath, nullptr, nullptr, &pathNum, MAX_POLYS, 0);

		for (int i = 0; i < pathNum && tmpPath.size() < (size_t)maxPoints * 3; i++) {
			tmpPath.emplace_back(detourPath[i * 3]);
			tmpPath.emplace_back(detourPath[i * 3 + 1]);
			tmpPath.emplace_back(detourPath[i * 3 + 2]);
		}

		if (isCompleted) { break; }
		startRef = queryPolysM[numPolys - 1];
		dtVcopy(start, epos);
	}

	path.clear();
	if (tmpPath.size() < 3*3) {
		path.assign(tmpPath.begin(), tmpPath.end());
		return true;
	}

	//go direct, drop point if it is bypass-able
	int startIndex = 0, nextIndex = 1;
	int tmpPointNum = (int)tmpPath.size() / 3;
	for (; nextIndex < (tmpPointNum - 1); nextIndex++) {
		float* p0 = tmpPath.data() + startIndex * 3;
		float* p1 = tmpPath.data() + nextIndex * 3;

		//drop p1 if p0 goes straightly to p2
		bool reachFarPoint = false;
		int testIndex = (int)(nextIndex + testAdvanceSteps) >= tmpPointNum ? (tmpPointNum - 1) : (nextIndex + testAdvanceSteps);
		for (; testIndex > nextIndex; testIndex--) {
			float* p2 = tmpPath.data() + testIndex * 3;
			float progress = raycast(p0, p2);
			if (progress >= 0.9999f) {
				reachFarPoint = true;
				nextIndex = testIndex - 1;
				break;
			}
		}
		if (reachFarPoint) {continue;}

		path.push_back(p0[0]);
		path.push_back(p0[1]);
		path.push_back(p0[2]);
		startIndex = nextIndex;
	}

	//push back test start point
	path.push_back(tmpPath[startIndex* 3 + 0]);
	path.push_back(tmpPath[startIndex* 3 + 1]);
	path.push_back(tmpPath[startIndex* 3 + 2]);

	//push back test next point if it is valid
	if (nextIndex < tmpPath.size() / 3) {
		path.push_back(tmpPath[nextIndex*3 + 0]);
		path.push_back(tmpPath[nextIndex*3 + 1]);
		path.push_back(tmpPath[nextIndex*3 + 2]);
	}

    return true;
}

bool NavMeshWrapper::closestPointOnMesh(float* testPoint, float* inMeshPoint) {
    dtPolyRef startRef = 0;

    dtStatus status = navQueryM->findNearestPoly(testPoint, queryExtM, &queryFilterM, &startRef, 0);
	if (dtStatusFailed(status)){
		return false;
    }

	navQueryM->closestPointOnPoly(startRef, testPoint, inMeshPoint, 0);
	return true;
}

float NavMeshWrapper::raycast(float* start, float *end)
{
    dtPolyRef startRef = 0;

    dtStatus status = navQueryM->findNearestPoly(start, queryExtM, &queryFilterM, &startRef, 0);
	if (dtStatusFailed(status)){
		return -1;
    }

	dtRaycastHit hit;
    status = navQueryM->raycast(startRef, start, end, &queryFilterM, 0, &hit);
	if (dtStatusFailed(status)){
		return -1;
    }
	if (hit.t >= 1.0f) { return 1; }

	return hit.t;
}

