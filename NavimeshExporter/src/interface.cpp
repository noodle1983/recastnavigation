#include "interface.h"

#include "MeshParser.hpp"
using namespace nd;

#include "nd_header.h"
namespace nd{
#include "Detour/DetourCommon.h"
#include "Detour/DetourNavMesh.h"
#include "Detour/DetourNavMeshBuilder.cpp"

	template<class T> inline T rcMin(T a, T b) { return a < b ? a : b; }
	inline unsigned int nextPow2(unsigned int v)
	{
		v--;
		v |= v >> 1;
		v |= v >> 2;
		v |= v >> 4;
		v |= v >> 8;
		v |= v >> 16;
		v++;
		return v;
	}
	inline unsigned int ilog2(unsigned int v)
	{
		unsigned int r;
		unsigned int shift;
		r = (v > 0xffff) << 4; v >>= r;
		shift = (v > 0xff) << 3; v >>= shift; r |= shift;
		shift = (v > 0xf) << 2; v >>= shift; r |= shift;
		shift = (v > 0x3) << 1; v >>= shift; r |= shift;
		r |= (v >> 1);
		return r;
	}
}

#include <cstring>
#include <memory>
using namespace std;

//[StructLayout(LayoutKind.Sequential)]
//class MyClass {
//  ...
//}
struct NavimeshTileData{
    float* verts;
    int vertCount;

    // per vertice/edge data
    unsigned short* triangles;
    unsigned short* neis;
    int triangleIndexCount;

    // per triangle data
    short* polyFlags;
    char* polyAreaTypes;

    // portal count
    short portalCount;

    bool buildBvTree;
	int tileX;
	int tileY;
	int tileLayer;
	unsigned int userId;

	float walkableHeight;
	float walkableRadius;
	float walkableClimb;
	float bmin[3];
	float bmax[3];
};

const float BvQuantFactor = 1;
static int createBVTree(NavimeshTileData* tileData, dtBVNode* nodes, int /*nnodes*/)
{
	// Build tree
    const int polyCount = tileData->triangleIndexCount/3;
	BVItem* items = (BVItem*)dtAlloc(sizeof(BVItem)*polyCount, DT_ALLOC_TEMP);
	float* tbmin = tileData->bmin;
	for (int i = 0; i < polyCount; i++)
	{
		BVItem& it = items[i];
		it.i = i;
		float bmin[3];
		float bmax[3];

        const unsigned short* p = &tileData->triangles[i*3];
		dtVcopy(bmin, &tileData->verts[p[0] * 3]);
		dtVcopy(bmax, &tileData->verts[p[0] * 3]);

        for (int j = 1; j < 3; ++j)
        {
			dtVmin(bmin, &tileData->verts[p[j] * 3]);
			dtVmax(bmax, &tileData->verts[p[j] * 3]);
        }
        it.bmin[0] = (unsigned short)dtMathFloorf((bmin[0]-tbmin[0])*BvQuantFactor);
        it.bmin[1] = (unsigned short)dtMathFloorf((bmin[1]-tbmin[1])*BvQuantFactor);
        it.bmin[2] = (unsigned short)dtMathFloorf((bmin[2]-tbmin[2])*BvQuantFactor);

        it.bmax[0] = (unsigned short)dtMathCeilf((bmax[0]-tbmin[0])*BvQuantFactor);
        it.bmax[1] = (unsigned short)dtMathCeilf((bmax[1]-tbmin[1])*BvQuantFactor);
        it.bmax[2] = (unsigned short)dtMathCeilf((bmax[2]-tbmin[2])*BvQuantFactor);
	}
	
	int curNode = 0;
	subdivide(items, polyCount, 0, polyCount, curNode, nodes);
	
	dtFree(items);
	
	return curNode;
}

bool dumpSoloTileData(NavimeshTileData* tileData, unsigned char** outData, int* outDataSize) {
	if (tileData->vertCount >= 0xffff)
		return false;
    if (!tileData->vertCount || !tileData->verts)
		return false;
	if (!tileData->triangleIndexCount || !tileData->triangles)
		return false;

	int storedOffMeshConCount = 0;
	int offMeshConLinkCount = 0;

	// Off-mesh connectionss are stored as polygons, adjust values.
    const int polyCount = tileData->triangleIndexCount/3;
	const int totPolyCount = tileData->triangleIndexCount/3 + storedOffMeshConCount;
	const int totVertCount = tileData->vertCount + storedOffMeshConCount*2;
    const int portalCount = tileData->portalCount;
    //const int edgeCount = tileData->triangleIndexCount;
    const int maxLinkCount = tileData->triangleIndexCount + tileData->portalCount*2 + offMeshConLinkCount*2;

	int uniqueDetailVertCount = 0;
	int detailTriCount = tileData->triangleIndexCount/3;

	// Calculate data size
	const int headerSize = dtAlign4(sizeof(dtMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float)*3*totVertCount);
	const int polysSize = dtAlign4(sizeof(dtPoly)*totPolyCount);
	const int linksSize = dtAlign4(sizeof(dtLink)*maxLinkCount);
	const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail)*polyCount);
	const int detailVertsSize = dtAlign4(sizeof(float)*3*uniqueDetailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char)*4*detailTriCount);
	const int bvTreeSize = tileData->buildBvTree ? dtAlign4(sizeof(dtBVNode)*polyCount*2) : 0;
	const int offMeshConsSize = dtAlign4(sizeof(dtOffMeshConnection)*storedOffMeshConCount);
	
	const int dataSize = headerSize + vertsSize + polysSize + linksSize +
						 detailMeshesSize + detailVertsSize + detailTrisSize +
						 bvTreeSize + offMeshConsSize;
						 
	unsigned char* data = (unsigned char*)dtAlloc(sizeof(unsigned char)*dataSize, DT_ALLOC_PERM);
	if (!data) { return false; }
	memset(data, 0, dataSize);

	unsigned char* d = data;
	dtMeshHeader* header = dtGetThenAdvanceBufferPointer<dtMeshHeader>(d, headerSize);
	float* navVerts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	dtPoly* navPolys = dtGetThenAdvanceBufferPointer<dtPoly>(d, polysSize);
	d += linksSize; // Ignore links; just leave enough space for them. They'll be created on load.
	dtPolyDetail* navDMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(d, detailMeshesSize);
	float* navDVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	unsigned char* navDTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
	dtBVNode* navBvtree = dtGetThenAdvanceBufferPointer<dtBVNode>(d, bvTreeSize);
	dtOffMeshConnection* offMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(d, offMeshConsSize);
	
	// Store header
	header->magic = DT_NAVMESH_MAGIC;
	header->version = DT_NAVMESH_VERSION;
	header->x = tileData->tileX;
	header->y = tileData->tileY;
	header->layer = tileData->tileLayer;
	header->userId = tileData->userId;
	header->polyCount = totPolyCount;
	header->vertCount = totVertCount;
	header->maxLinkCount = maxLinkCount;
	dtVcopy(header->bmin, tileData->bmin);
	dtVcopy(header->bmax, tileData->bmax);
	header->detailMeshCount = polyCount;
	header->detailVertCount = uniqueDetailVertCount;
	header->detailTriCount = detailTriCount;
	header->bvQuantFactor = BvQuantFactor;   // limited scene max length to 0xffff
	header->offMeshBase = polyCount;
	header->walkableHeight = tileData->walkableHeight;
	header->walkableRadius = tileData->walkableRadius;
	header->walkableClimb = tileData->walkableClimb;
	header->offMeshConCount = storedOffMeshConCount;
	header->bvNodeCount = tileData->buildBvTree ? polyCount*2 : 0;
	
	const int offMeshVertsBase = tileData->vertCount;
	const int offMeshPolyBase = polyCount;
	
	// Store vertices
	// Mesh vertices
    memcpy(navVerts, tileData->verts, sizeof(tileData->verts[0]) * tileData->vertCount);

	// Store polygons
	// Mesh polys
	for (int i = 0; i < polyCount; ++i)
	{
		dtPoly* p = &navPolys[i];
		p->vertCount = 3;
		p->flags = tileData->polyFlags[i];
		p->areaAndtype = tileData->polyAreaTypes[i];
        for(int j = 0; j < p->vertCount; j++){
            p->verts[j] = tileData->triangles[i*3 + j];
            p->neis[j] = tileData->neis[i*3 + j];
        }
	}

	// Store detail meshes and vertices.
	// The nav polygon vertices are stored as the first vertices on each mesh.
	// We compress the mesh data by skipping them and using the navmesh coordinates.
    // Create dummy detail mesh by triangulating polys.
    int tbase = 0;
    for (int i = 0; i < polyCount; ++i)
    {
        dtPolyDetail& dtl = navDMeshes[i];
        const int nv = navPolys[i].vertCount;
        dtl.vertBase = 0;
        dtl.vertCount = 0;
        dtl.triBase = (unsigned int)tbase;
        dtl.triCount = (unsigned char)(nv-2);
        // Triangulate polygon (local indices).
        for (int j = 2; j < nv; ++j)
        {
            unsigned char* t = &navDTris[tbase*4];
            t[0] = 0;
            t[1] = (unsigned char)(j-1);
            t[2] = (unsigned char)j;
            // Bit for each edge that belongs to poly boundary.
            t[3] = (1<<2);
            if (j == 2) t[3] |= (1<<0);
            if (j == nv-1) t[3] |= (1<<4);
            tbase++;
        }
    }

	// Store and create BVtree.
	if (tileData->buildBvTree)
	{
		createBVTree(tileData, navBvtree, 2*polyCount);
	}
	
	*outData = data;
	*outDataSize = dataSize;
	
	return true;
}

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

void saveAll(const char* path, const dtNavMesh* mesh)
{
	if (!mesh) return;

	FILE* fp = fopen(path, "wb");
	if (!fp)
		return;

	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);
}

static char* dupstr(const char* str) {
	if (str == nullptr) { return nullptr; }
	int len = (int)strlen(str) + 1;
	char* ret = new char[len];
	if (len > 1) { memcpy(ret, str, len - 1); }
	ret[len - 1] = 0;
	return ret;
}

char* exportDetourFormatFile(const char* detourMeshPath, const char* detourBinPath){
    auto mesh = parseMesh(detourMeshPath);
	if (mesh == nullptr) { return dupstr("failed to parse the detour mesh file!"); }
	shared_ptr<Mesh> meshDeletor(mesh);

	auto& vertices = mesh->vertices;
	auto& vi = mesh->triangles;
	auto& trianglesFlag = mesh->trianglesFlag;
	auto& trianglesAreaType = mesh->trianglesAreaType;
	auto& lineNeis = mesh->lineNeis;

	if (vertices.size() == 0 || vi.size() == 0) { return dupstr("empty mesh!"); }

    struct NavimeshTileData tileData;
    memset(&tileData, 0, sizeof(NavimeshTileData));

    tileData.verts = vertices.data();
    tileData.vertCount = (int)vertices.size();
	float* bmin = tileData.bmin;
	float* bmax = tileData.bmax;
	dtVcopy(bmin, tileData.verts);
	dtVcopy(bmax, tileData.verts);
    for(int i = 0; i < tileData.vertCount/3; i++){
        float* base = &tileData.verts[i*3];
		dtVmin(bmin, base);
		dtVmax(bmax, base);
    }

    // per vertice/edge data
    tileData.triangles = vi.data();
    tileData.triangleIndexCount = (int)vi.size();
    using Edge2Triangle = map<uint64_t, vector<unsigned short>>;
    Edge2Triangle edge2triangle;
    auto make_line = [](uint64_t a, uint64_t b){
        return a > b ? ((b<<32) | a) : ((a<<32) | b);
    };
    for(unsigned short i = 0; i < vi.size()/3; i++){
        unsigned short* base = &vi[i * 3];
        uint64_t line1 = make_line(base[0], base[1]);
        edge2triangle[line1].push_back(i);

        uint64_t line2 = make_line(base[1], base[2]);
        edge2triangle[line2].push_back(i);

        uint64_t line3 = make_line(base[2], base[0]);
        edge2triangle[line3].push_back(i);
    }

    VerticeIndexes edgeInfo;
    VerticeIndexes flagsInfo;
    using CharVector = vector<char>;
    CharVector areaTypesInfo;
    for(unsigned short i = 0; i < vi.size()/3; i++){
        flagsInfo.push_back(trianglesFlag[i]);
        areaTypesInfo.push_back(trianglesAreaType[i]);

        unsigned short* base = &vi[i * 3];
        for(int j = 0; j < 3; j++){
            uint64_t line = make_line(base[j], base[(j+1)%3]);
            if(edge2triangle[line].size() == 1){
                edgeInfo.push_back(0);
            }else{
                unsigned short otherPolyRef = edge2triangle[line][0] == i ? edge2triangle[line][1] : edge2triangle[line][0];
                edgeInfo.push_back(otherPolyRef + 1);
            }
        }
    }
    tileData.neis = edgeInfo.data();

    // per triangle data
    tileData.polyFlags = (short*)flagsInfo.data();
    tileData.polyAreaTypes = areaTypesInfo.data();

    // portal count
    tileData.portalCount = 0;

    tileData.buildBvTree = true;
	tileData.tileX = 0;
	tileData.tileY = 0;
	tileData.tileLayer = 0;
	tileData.userId = 0;

	tileData.walkableHeight = 0;
	tileData.walkableRadius = 0;
	tileData.walkableClimb = 0;

    unsigned char* outData = NULL;
    int outDataSize = 0;
    bool ret = dumpSoloTileData(&tileData, &outData, &outDataSize);
    if(ret){
		dtNavMesh* navMesh = dtAllocNavMesh();
		if (!navMesh)
		{
			dtFree(outData);
			return dupstr("Could not create Detour navmesh");
		}
		shared_ptr<dtNavMesh> navMeshDeletor(navMesh, [](auto p) {dtFreeNavMesh(p); });

		dtStatus status = navMesh->init(outData, outDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
		{
			dtFree(outData);
			return dupstr("Could not init Detour navmesh");
		}

        saveAll(detourBinPath, navMesh);
    }
    return nullptr;
}

unsigned char* buildTileMesh(TiledMesh* mesh, int& dataSize){
	auto& vertices = mesh->vertices;
	auto& vi = mesh->triangles;
	auto& trianglesFlag = mesh->trianglesFlag;
	auto& trianglesAreaType = mesh->trianglesAreaType;
	auto& lineNeis = mesh->lineNeis;
	auto tx = mesh->tx;
	auto ty = mesh->ty;

    struct NavimeshTileData tileData;
    memset(&tileData, 0, sizeof(NavimeshTileData));

    tileData.verts = vertices.data();
    tileData.vertCount = (int)vertices.size();
	float* bmin = tileData.bmin;
	float* bmax = tileData.bmax;
	dtVcopy(bmin, tileData.verts);
	dtVcopy(bmax, tileData.verts);
    for(int i = 0; i < tileData.vertCount/3; i++){
        float* base = &tileData.verts[i*3];
		dtVmin(bmin, base);
		dtVmax(bmax, base);
    }

    // per vertice/edge data
    tileData.triangles = vi.data();
    tileData.triangleIndexCount = (int)vi.size();
    using Edge2Triangle = map<uint64_t, vector<unsigned short>>;
    Edge2Triangle edge2triangle;
    auto make_line = [](uint64_t a, uint64_t b){
        return a > b ? ((b<<32) | a) : ((a<<32) | b);
    };
    for(unsigned short i = 0; i < vi.size()/3; i++){
        unsigned short* base = &vi[i * 3];
        uint64_t line1 = make_line(base[0], base[1]);
        edge2triangle[line1].push_back(i);

        uint64_t line2 = make_line(base[1], base[2]);
        edge2triangle[line2].push_back(i);

        uint64_t line3 = make_line(base[2], base[0]);
        edge2triangle[line3].push_back(i);
    }

    VerticeIndexes edgeInfo;
    VerticeIndexes flagsInfo;
    using CharVector = vector<char>;
    CharVector areaTypesInfo;
    CharVector typeInfo;    
	short portalCount = 0;
    for(unsigned short i = 0; i < vi.size()/3; i++){
        flagsInfo.push_back(trianglesFlag[i]);
        areaTypesInfo.push_back(trianglesAreaType[i]);

        unsigned short* base = &vi[i * 3];
		unsigned short* lineNeisBase = &lineNeis[i * 3];
        for(int j = 0; j < 3; j++){
            uint64_t line = make_line(base[j], base[(j+1)%3]);
			if (lineNeisBase[j] & DT_EXT_LINK) {
                edgeInfo.push_back(lineNeisBase[j]);
				portalCount++;
			}
            else if(edge2triangle[line].size() == 1){
                edgeInfo.push_back(0);
            }else{
                unsigned short otherPolyRef = edge2triangle[line][0] == i ? edge2triangle[line][1] : edge2triangle[line][0];
                edgeInfo.push_back(otherPolyRef + 1);
            }
        }
    }
    tileData.neis = edgeInfo.data();

    // per triangle data
    tileData.polyFlags = (short*)flagsInfo.data();
    tileData.polyAreaTypes = areaTypesInfo.data();

    // portal count
	tileData.portalCount = portalCount;

    tileData.buildBvTree = true;
	tileData.tileX = tx;
	tileData.tileY = ty;
	tileData.tileLayer = 0;
	tileData.userId = 0;

	tileData.walkableHeight = 0;
	tileData.walkableRadius = 0;
	tileData.walkableClimb = 0;

    unsigned char* outData = NULL;
    bool ret = dumpSoloTileData(&tileData, &outData, &dataSize);
    return outData;
}


char* exportTiledDetourFormatFile(const char* detourMeshPath, const char* detourBinPath) {
	auto mesh = parseMesh(detourMeshPath);
	if (mesh == nullptr) { return dupstr("failed to parse the detour mesh file!"); }

	shared_ptr<Mesh> meshDeletor(mesh);
	if (mesh->tiledMeshList.size() <= 1) {
		return exportDetourFormatFile(detourMeshPath, detourBinPath);
	}

	dtNavMesh* navMesh = dtAllocNavMesh();
	if (!navMesh) { return dupstr("Could not create Detour navmesh"); }
	shared_ptr<dtNavMesh> navMeshDeletor(navMesh, [](auto p) {dtFreeNavMesh(p); });

	dtNavMeshParams params;
	dtVcopy(params.orig, mesh->bmin.data());
	params.tileWidth = mesh->tileWidth;
	params.tileHeight = mesh->tileHeight;

	int tileBits = ilog2(nextPow2(mesh->tw* mesh->th));
	if (tileBits > 14) { return dupstr("too many tiles!"); }

	int polyBits = 22 - tileBits;
	params.maxTiles = 1 << tileBits;
	params.maxPolys = 1 << polyBits;
	
	dtStatus status = navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		return dupstr("Could not init Detour navmesh");
	}

	for (auto & tiledMesh : mesh->tiledMeshList)
	{
		int polyCount = (int)tiledMesh.triangles.size() / 3;
		if (polyCount > params.maxPolys) { return dupstr("too many polys in a tile!"); }
		if (tiledMesh.vertices.size() == 0 || tiledMesh.triangles.size() == 0) { continue; }

		int dataSize = 0;
		unsigned char* data = buildTileMesh(&tiledMesh, dataSize);
		if (data)
		{
			// Remove any previous data (navmesh owns and deletes the data).
			navMesh->removeTile(navMesh->getTileRefAt(tiledMesh.tx, tiledMesh.ty, 0), 0, 0);
			// Let the navmesh own the data.
			dtStatus status = navMesh->addTile(data, dataSize, DT_TILE_FREE_DATA, 0, 0);
			if (dtStatusFailed(status)) { dtFree(data); }
		}
	}

	saveAll(detourBinPath, navMesh);
	return nullptr;
}

