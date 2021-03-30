#ifndef MESHPARSER_H
#define MESHPARSER_H

#include "rapidjson/filereadstream.h"
#include "rapidjson/error/en.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "rapidjson/document.h"
using namespace rapidjson;

#include <iostream>
#include <stdint.h>
#include <vector>
#include <array>
#include <set>
#include <map>
#include <algorithm>
#include <cstring>
#include <limits>
#include <tuple>
using namespace std;


namespace nd{
    using Vertices = std::vector<float>;
    using LineNeis = std::vector<unsigned short>;
    using Triangles = std::vector<unsigned short>;
	using TrianglesFlag = std::vector<unsigned>;
	using TrianglesAreaType = std::vector<uint8_t>;
    using Normals = std::vector<float>;
    using Vector3 = std::array<float, 3>;
    using TriVector3 = std::array<Vector3, 3>;
	using VerticeIndexes = std::vector<unsigned short>;

	enum SamplePolyFlags
	{
		SAMPLE_POLYFLAGS_WALK = 0x01,       // Ability to walk (ground, grass, road)
		SAMPLE_POLYFLAGS_SWIM = 0x02,       // Ability to swim (water).
		SAMPLE_POLYFLAGS_DOOR = 0x04,       // Ability to move through doors.
		SAMPLE_POLYFLAGS_JUMP = 0x08,       // Ability to jump.
		SAMPLE_POLYFLAGS_DISABLED = 0x10,       // Disabled polygon
		SAMPLE_POLYFLAGS_ALL = 0xffff   // All abilities.
	};

	struct TiledMesh {
		int tx;
		int ty;
        Vertices vertices;
        Triangles triangles;
        Normals normals;
		TrianglesFlag trianglesFlag;
		TrianglesAreaType trianglesAreaType;
		LineNeis lineNeis;
	};

    using TiledMeshList = std::vector<TiledMesh>;

    struct Mesh{
        Vertices vertices;
        Triangles triangles;
        Normals normals;
		TrianglesFlag trianglesFlag;
		TrianglesAreaType trianglesAreaType;
		LineNeis lineNeis;
		unsigned navFlag;
		unsigned navAreaType;
		Vector3 bmin;
		Vector3 bmax;

		unsigned tw;
		unsigned th;
		float tileWidth;
		float tileHeight;
		TiledMeshList tiledMeshList;

        Mesh(const Mesh& other) = default;
        Mesh() = default;

        void MoveFrom(Mesh* m) {
            Clear();
            vertices = std::move(m->vertices);
            triangles = std::move(m->triangles);
            normals = std::move(m->normals);
            trianglesFlag = std::move(m->trianglesFlag);
            trianglesAreaType = std::move(m->trianglesAreaType);
            lineNeis = std::move(m->lineNeis);
            navFlag = m->navFlag;
            navAreaType = m->navAreaType;
            delete m;
        }

        void Clear() { 
			vertices.clear(); triangles.clear(); normals.clear();
            trianglesFlag.clear(); trianglesAreaType.clear(); lineNeis.clear();
		}

        Vector3 GetPoint(int index) { 
            if (index < 0 || (index*3+2) >= (int)vertices.size()){return {0,0,0};}
            return { vertices[index*3 + 0], vertices[index*3 + 1], vertices[index*3 + 2] };
        }
        Vector3 GetNormal(int index) {
            if (index < 0 || (index*3+2) >= (int)normals.size()){return {0,1,0};}
            return { normals[index*3 + 0], normals[index*3 + 1], normals[index*3 + 2] };
        }
        void UpdateVetice(const int index, const Vector3& v){
            vertices[index * 3 + 0]  = v[0];
            vertices[index * 3 + 1]  = v[1];
            vertices[index * 3 + 2]  = v[2];
        }
        int AddExtraPoint(const Vector3& crosingPoint, const Vector3& normal){
            int index = (int)vertices.size()/3;
            vertices.push_back(crosingPoint[0]);
            vertices.push_back(crosingPoint[1]);
            vertices.push_back(crosingPoint[2]);

            normals.push_back(normal[0]);
            normals.push_back(normal[1]);
            normals.push_back(normal[2]);
            return index;
        }

        unsigned GetTriangleFlag(unsigned triangleIndex) {
            if (trianglesFlag.size() > triangleIndex) {
                return trianglesFlag[triangleIndex];
            }
            return navFlag;
        }

        unsigned char GetTriangleAreaType(unsigned triangleIndex) {
            if (trianglesAreaType.size() > triangleIndex) {
                return trianglesAreaType[triangleIndex];
            }
            return navAreaType;
        }

        uint16_t GetLineFlags(unsigned lineIndex) {
            if (lineNeis.size() > lineIndex) {
                return  lineNeis[lineIndex];
            }
            return 0;
        }

    };


	Document* parseFile(const char* path) {
		FILE* fp = fopen(path, "r");
		if (fp == NULL) {
			cerr << "failed to open file:" << path << ", errno:" << errno << endl;
			return NULL;
		}

		char readBuffer[65536];
		FileReadStream is(fp, readBuffer, sizeof(readBuffer));
		auto doc = new Document();
		ParseResult ok = doc->ParseStream(is);
		if (!ok) {
			fclose(fp);
			cerr << "JSON parse error: " << GetParseError_En(ok.Code()) << ", offset: " << ok.Offset() << endl;
			return NULL;
		}
		fclose(fp);
		return doc;
	}

	float* parseVector(rapidjson::Value& value, float* v, float* offset) {
		v[0] = value["x"].GetFloat() + (offset ? offset[0] : 0);
		v[1] = value["y"].GetFloat() + (offset ? offset[1] : 0);
		v[2] = value["z"].GetFloat() + (offset ? offset[2] : 0);
		return v;
	}

	void logVector(float* v) {
		cout << "x=" << v[0] << ", y=" << v[1] << ", z=" << v[2] << endl;
	}

	void parseVertices(rapidjson::Value& value, Vertices& v, float* offset) {
		v.clear();
		SizeType size = value.Size();
		v.resize(size * 3);
		for (SizeType i = 0; i < size; i++) {
			parseVector(value[i], &v[i * 3], offset);
		}
	}

	void logVertices(Vertices& v) {
		for (unsigned i = 0; i < v.size() / 3; i++) {
			cout << "Vertices[" << i << "]: ";
			logVector(&v[i * 3]);
		}
	}

	void parseVerticeIndexes(rapidjson::Value& value, Triangles& vi) {
		vi.clear();
		SizeType size = value.Size();
		vi.resize(size);
		for (SizeType i = 0; i < size; i++) {
			vi[i] = value[i].GetUint();
		}
	}

	void logVerticeIndexes(Triangles& v) {
		cout << "VerticeIndexes:";
		for (unsigned i = 0; i < v.size(); i++) {
			cout << " " << v[i];
		}
		cout << endl;
	}

	void parseTrianglesFlag(rapidjson::Value& value, TrianglesFlag& ta) {
		ta.clear();
		SizeType size = value.Size();
		ta.resize(size);
		for (SizeType i = 0; i < size; i++) {
			ta[i] = value[i].GetUint();
		}
	}

	void parseTrianglesArea(rapidjson::Value& value, TrianglesAreaType& ta) {
		ta.clear();
		SizeType size = value.Size();
		ta.resize(size);
		for (SizeType i = 0; i < size; i++) {
			ta[i] = (uint8_t)value[i].GetUint();
		}
	}

	void parseLineNeis(rapidjson::Value& value, LineNeis& ln) {
		ln.clear();
		SizeType size = value.Size();
		ln.resize(size);
		for (SizeType i = 0; i < size; i++) {
			ln[i] = value[i].GetUint();
		}
	}

	void parseTiledMesh(rapidjson::Value& value, TiledMesh& tml) {
		tml.tx = value["tx"].GetInt();
		tml.ty = value["ty"].GetInt();
		parseVertices(value["vertices"], tml.vertices, nullptr);
		parseVerticeIndexes(value["triangles"], tml.triangles);
		parseVertices(value["normals"], tml.normals, nullptr);
		parseTrianglesFlag(value["trianglesFlag"], tml.trianglesFlag);
		parseTrianglesArea(value["trianglesAreaType"], tml.trianglesAreaType);
		parseLineNeis(value["exLinesFlag"], tml.lineNeis); 
	}

	void parseTiledMeshList(rapidjson::Value& value, TiledMeshList& tml) {
		tml.clear();
		SizeType size = value.Size();
		tml.resize(size);
		for (SizeType i = 0; i < size; i++) {
			parseTiledMesh(value[i], tml[i]);
		}
	}

	Mesh* parseMesh(const char* path) {
		auto doc = parseFile(path);
		if (doc == nullptr) { return nullptr; }

		Document& data = *doc;
		float offset[3];
		parseVector(data["offset"], offset, nullptr);

		Mesh* mesh = new Mesh();
		mesh->tw = data["tw"].GetInt();
		mesh->th = data["th"].GetInt();
		mesh->tileWidth = data["tileWidth"].GetFloat();
		mesh->tileHeight = data["tileHeight"].GetFloat();
		parseVector(data["bmin"], mesh->bmin.data(), nullptr);
		parseVector(data["bmax"], mesh->bmax.data(), nullptr);
		parseVertices(data["simpleMesh"]["vertices"], mesh->vertices, offset);
		parseVerticeIndexes(data["simpleMesh"]["triangles"], mesh->triangles);
		parseVertices(data["simpleMesh"]["normals"], mesh->normals, nullptr);
		mesh->navFlag = data["navigationFlag"].GetUint();
		mesh->navAreaType = data["navigationAreaType"].GetUint();
		if (data.HasMember("trianglesFlag")) { parseTrianglesFlag(data["trianglesFlag"], mesh->trianglesFlag); }
		if (data.HasMember("trianglesAreaType")) { parseTrianglesArea(data["trianglesAreaType"], mesh->trianglesAreaType); }
		if (data.HasMember("exLinesFlag")) { parseLineNeis(data["exLinesFlag"], mesh->lineNeis); }
		if (data.HasMember("tiledMeshes")) { parseTiledMeshList(data["tiledMeshes"], mesh->tiledMeshList); }

		delete doc;
		return mesh;
	}

}
#endif /* MESHPARSER_H */
