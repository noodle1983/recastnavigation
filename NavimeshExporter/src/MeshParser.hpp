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
    using LineNeis = std::vector<short>;
    using Triangles = std::vector<unsigned short>;
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


    struct Mesh{
        Vertices vertices;
        Triangles triangles;
        Normals normals;

        Mesh(const Mesh& other) = default;
        Mesh() = default;

        void MoveFrom(Mesh* m) {
            Clear();
            vertices = std::move(m->vertices);
            triangles = std::move(m->triangles);
            normals = std::move(m->normals);
            delete m;
        }

        void Clear() { vertices.clear(); triangles.clear(); normals.clear(); }

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


	Mesh* parseMesh(const char* path) {
		auto doc = parseFile(path);
		if (doc == nullptr) { return nullptr; }

		Document& data = *doc;
		float offset[3];
		parseVector(data["offset"], offset, nullptr);

		Mesh* mesh = new Mesh();
		parseVertices(data["simpleMesh"]["vertices"], mesh->vertices, offset);
		parseVerticeIndexes(data["simpleMesh"]["triangles"], mesh->triangles);
		parseVertices(data["simpleMesh"]["normals"], mesh->normals, nullptr);

		delete doc;
		return mesh;
	}

}
#endif /* MESHPARSER_H */
