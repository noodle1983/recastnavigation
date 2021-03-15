#ifndef INTERFACE_H
#define INTERFACE_H

#if defined (_MSC_VER)
#define LIB_API __declspec(dllexport)
#else
#define LIB_API
#endif

extern "C" {
	LIB_API char* exportDetourFormatFile(const char* detourMeshPath, const char* detourBinPath);
};

#endif
