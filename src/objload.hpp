#pragma once

#include <array>

#include "geometry.hpp"
#include "tiny_obj_loader.h"
namespace rt {
	struct TriangleFace {
		rt::Vec3 v[3];
		rt::Vec3 n[3];
		rt::Vec2 t[3];

		rt::Vec3 tangent[3];
		rt::Vec3 bitangent[3];
	};
	inline std::vector<TriangleFace> loadObjWithTangent(std::string inputfile)
	{
		tinyobj::attrib_t attrib;
		std::vector<tinyobj::shape_t> shapes;
		std::vector<tinyobj::material_t> materials;

		std::string err;
		bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, inputfile.c_str());

		if (!err.empty()) { // `err` may contain warning message.
			std::cerr << err << std::endl;
		}

		std::vector<TriangleFace> triangles;

		for (size_t s = 0; s < shapes.size(); s++) {
			size_t index_offset = 0;
			for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
				int fv = shapes[s].mesh.num_face_vertices[f];
				assert(fv == 3);

				TriangleFace tri;
				for (size_t v = 0; v < fv; v++) {
					tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
					for (int i = 0; i < 3; ++i) {
						tri.v[v][i] = attrib.vertices[3 * idx.vertex_index + i];
						tri.n[v][i] = attrib.normals[3 * idx.normal_index + i];
					}
					for (int i = 0; i < 2; ++i) {
						tri.t[v][i] = attrib.texcoords[2 * idx.texcoord_index + i];
					}
				}
				index_offset += fv;

				triangles.push_back(tri);
			}
			break;
		}

		// calc tangent, bitangent
		for (int i = 0; i < triangles.size(); ++i) {
			auto tri = triangles[i];

			Vec3 deltaPos1 = tri.v[1] - tri.v[0];
			Vec3 deltaPos2 = tri.v[2] - tri.v[0];

			Vec2 deltaUV1 = tri.t[1] - tri.t[0];
			Vec2 deltaUV2 = tri.t[2] - tri.t[0];

			double r = 1.0 / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);
			Vec3 tangent = (deltaPos1 * deltaUV2.y - deltaPos2 * deltaUV1.y) * r;
			Vec3 bitangent = (deltaPos2 * deltaUV1.x - deltaPos1 * deltaUV2.x) * r;
			tangent = glm::normalize(tangent);
			bitangent = glm::normalize(bitangent);

			for (int j = 0; j < 3; ++j) {
				tri.tangent[j] = tangent;
				tri.bitangent[j] = bitangent;
			}
			triangles[i] = tri;
		}
		return triangles;
	}
}