#ifndef ND_HEADER_H
#define ND_HEADER_H

#include <stdlib.h>
#include <stddef.h>
#include <assert.h> 
#include <math.h>
#include <cmath>
#include <stdint.h>
#include <float.h>
#include <string.h>
#include <new>
#include <stdio.h>

#if defined (_MSC_VER)
static inline FILE* my_fopen(char const* _FileName, char const* _Mode) { FILE* fp = NULL; fopen_s(&fp, _FileName, _Mode); return fp; }
#define fopen my_fopen
#endif


#endif
