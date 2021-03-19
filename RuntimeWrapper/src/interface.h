#ifndef INTERFACE_H
#define INTERFACE_H

#if defined (_MSC_VER)
#define LIB_API __declspec(dllexport)
#else
#define LIB_API
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
	LIB_API float nd_raycast(void* p, float* start, float *end);
};

#endif
