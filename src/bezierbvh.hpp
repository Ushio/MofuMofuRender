#pragma once

#include <vector>
#include <immintrin.h>
#include <mapbox/variant.hpp>
#include <optional.hpp>

#include "geometry.hpp"
#include "peseudo_random.hpp"
#include "aabb.hpp"
#include "bezier.hpp"

#include <tbb/tbb.h>

namespace rt {
	// static double kBVH_SAH_POWER = 3.0;
	static double kBVH_SAH_POWER = 30.0;
	//struct AABBx4 {
	//	AABBx4() {}
	//	AABBx4(const AABB *aabbs) {
	//		for (int i = 0; i < 4; ++i) {
	//			min_position_x[i] = aabbs[i].min_position[0];
	//			min_position_y[i] = aabbs[i].min_position[1];
	//			min_position_z[i] = aabbs[i].min_position[2];
	//			max_position_x[i] = aabbs[i].max_position[0];
	//			max_position_y[i] = aabbs[i].max_position[1];
	//			max_position_z[i] = aabbs[i].max_position[2];

	//			_aabbs[i] = aabbs[i];
	//		}
	//	}
	//	alignas(alignof(__m128)) float min_position_x[4];
	//	alignas(alignof(__m128)) float min_position_y[4];
	//	alignas(alignof(__m128)) float min_position_z[4];
	//	alignas(alignof(__m128)) float max_position_x[4];
	//	alignas(alignof(__m128)) float max_position_y[4];
	//	alignas(alignof(__m128)) float max_position_z[4];

	//	AABB _aabbs[4];
	//};

	//struct AABBIntersection_x4 {
	//	std::experimental::optional<float> tmins[4];
	//};

	//inline AABBIntersection_x4 intersect_aabb_4x_simd(const Ray &ray, const rt::Vec3 &inv_d, const AABBx4& aabb) {
	//	__m128 inv_d_x = _mm_set1_ps(inv_d.x);
	//	__m128 inv_d_y = _mm_set1_ps(inv_d.y);
	//	__m128 inv_d_z = _mm_set1_ps(inv_d.z);
	//	__m128 ray_o_x = _mm_set1_ps(ray.o.x);
	//	__m128 ray_o_y = _mm_set1_ps(ray.o.y);
	//	__m128 ray_o_z = _mm_set1_ps(ray.o.z);

	//	__m128 tx1 = _mm_mul_ps(_mm_sub_ps(*(__m128 *)aabb.min_position_x, ray_o_x), inv_d_x);
	//	__m128 tx2 = _mm_mul_ps(_mm_sub_ps(*(__m128 *)aabb.max_position_x, ray_o_x), inv_d_x);
	//	__m128 tmin = _mm_min_ps(tx1, tx2);
	//	__m128 tmax = _mm_max_ps(tx1, tx2);

	//	__m128 ty1 = _mm_mul_ps(_mm_sub_ps(*(__m128 *)aabb.min_position_y, ray_o_y), inv_d_y);
	//	__m128 ty2 = _mm_mul_ps(_mm_sub_ps(*(__m128 *)aabb.max_position_y, ray_o_y), inv_d_y);
	//	tmin = _mm_max_ps(tmin, _mm_min_ps(ty1, ty2));
	//	tmax = _mm_min_ps(tmax, _mm_max_ps(ty1, ty2));

	//	__m128 tz1 = _mm_mul_ps(_mm_sub_ps(*(__m128 *)aabb.min_position_z, ray_o_z), inv_d_z);
	//	__m128 tz2 = _mm_mul_ps(_mm_sub_ps(*(__m128 *)aabb.max_position_z, ray_o_z), inv_d_z);
	//	tmin = _mm_max_ps(tmin, _mm_min_ps(tz1, tz2));
	//	tmax = _mm_min_ps(tmax, _mm_max_ps(tz1, tz2));

	//	/*
	//	bool intersect = tmin <= tmax && 0.0 < tmax;
	//	if (intersect) {
	//		*tmin_ptr = tmin < 0.0 ? tmax : tmin;
	//	}
	//	*/
	//	__m128 a = _mm_cmp_ps(tmin, tmax, _CMP_LE_OQ);
	//	__m128 b = _mm_cmp_ps(_mm_setzero_ps(), tmax, _CMP_LE_OQ);
	//	__m128 r = _mm_and_ps(a, b);
	//	__m128 tmin_final = _mm_blendv_ps(tmin, tmax, _mm_cmp_ps(tmin, _mm_setzero_ps(), _CMP_LE_OQ));

	//	AABBIntersection_x4 intersections;
	//	for (int i = 0; i < 4; ++i) {
	//		if (((uint32_t *)&r)[i]) {
	//			intersections.tmins[i] = ((float *)&tmin_final)[i];
	//		}
	//	}
	//	return intersections;
	//}
	//
	//inline AABBIntersection_x4 intersect_aabb_x4(const Ray &ray, const rt::Vec3 &inv_d, const AABB *aabb) {
	//	AABBIntersection_x4 r;
	//	for (int i = 0; i < 4; ++i) {
	//		if (auto tmin = intersect_aabb(ray, inv_d, aabb[i])) {
	//			r.tmins[i] = *tmin;
	//		}
	//	}
	//	return r;
	//}

	struct BezierEntity {
		double radius = 0.0;
		Vec3 sigma_a = Vec3();
		BezierQuadratic3D bezier;
	};

	/*
	基本となる木構造
	BVHNode = BVHLeaf | BVHBranch
	*/
	struct BVHLeaf {
		std::vector<int> bezierIndices;
	};

	struct BVHBranch;

	using BVHNode = mapbox::util::variant<BVHLeaf, std::unique_ptr<BVHBranch>>;

	struct BVHBranch {
		AABB aabb_L;
		AABB aabb_R;
		BVHNode lhs;
		BVHNode rhs;
	};

	/*
	QBVH 木構造
	QBVHNode = BVHLeaf | QBVHBranchx2 | QBVHBranchx4
	*/
	//struct QBVHBranchx2;
	//struct QBVHBranchx4;
	//using QBVHNode = mapbox::util::variant<BVHLeaf, std::unique_ptr<QBVHBranchx2>, std::unique_ptr<QBVHBranchx4>>;
	//struct QBVHBranchx2 {
	//	AABBx4 aabb_x4;
	//	QBVHNode nodes[2];
	//};
	//struct QBVHBranchx4 {
	//	AABBx4 aabb_x4;
	//	QBVHNode nodes[4];
	//};

	//inline QBVHNode to_qbvh(const BVHNode &bvhNode) {
	//	if (bvhNode.is<BVHLeaf>()) {
	//		return bvhNode.get<BVHLeaf>();
	//	}
	//	const std::unique_ptr<BVHBranch> &branch = bvhNode.get<std::unique_ptr<BVHBranch>>();
	//	bool isLBranch = branch->lhs.is<std::unique_ptr<BVHBranch>>();
	//	bool isRBranch = branch->rhs.is<std::unique_ptr<BVHBranch>>();
	//	if (isLBranch && isRBranch) {
	//		const std::unique_ptr<BVHBranch> &branchL = branch->lhs.get<std::unique_ptr<BVHBranch>>();
	//		const std::unique_ptr<BVHBranch> &branchR = branch->rhs.get<std::unique_ptr<BVHBranch>>();

	//		AABB aabbs[4] = {
	//			branchL->aabb_L,
	//			branchL->aabb_R,
	//			branchR->aabb_L,
	//			branchR->aabb_R,
	//		};
	//		std::unique_ptr<QBVHBranchx4> qBranch(new QBVHBranchx4());
	//		qBranch->aabb_x4 = AABBx4(aabbs);
	//		qBranch->nodes[0] = to_qbvh(branchL->lhs);
	//		qBranch->nodes[1] = to_qbvh(branchL->rhs);
	//		qBranch->nodes[2] = to_qbvh(branchR->lhs);
	//		qBranch->nodes[3] = to_qbvh(branchR->rhs);
	//		return qBranch;
	//	}
	//	std::unique_ptr<QBVHBranchx2> branch_x2(new QBVHBranchx2());
	//	AABB aabbs[4] = {
	//		branch->aabb_L,
	//		branch->aabb_R,
	//		branch->aabb_L,
	//		branch->aabb_R,
	//	};
	//	branch_x2->aabb_x4 = AABBx4(aabbs);
	//	branch_x2->nodes[0] = to_qbvh(branch->lhs);
	//	branch_x2->nodes[1] = to_qbvh(branch->rhs);
	//	return branch_x2;
	//}


	static const double kCOST_INTERSECT_AABB = 1.0;
	static const double kCOST_INTERSECT_TRIANGLE = 2.0;

	// 正規化し、連続化されたコクの有る乱数
	inline double koku(double nf, PeseudoRandom *random) {
		nf = std::max(1.0, nf);
		int n = std::floor(nf);
		double a = nf - n;

		double s = 0.0;
		for (int i = 0; i < n; ++i) {
			s += random->uniform(-1.0, 1.0);
		}
		s += random->uniform(-1.0, 1.0) * a;

		double sigma2 = n / 3.0 + a * a / 3.0;
		return s / std::sqrt(sigma2);
	}

	inline BVHNode build_tree(const std::vector<int> &bezierIndices, const BezierEntity *beziers, PeseudoRandom *random, int depth = 0) {
		int maxTriangles = 1;
		if (bezierIndices.size() <= maxTriangles) {
			BVHLeaf leaf;
			leaf.bezierIndices = bezierIndices;
			return leaf;
		}

		AABB aabb;
		for (int i = 0; i < bezierIndices.size(); ++i) {
			auto bezierEntity = beziers[bezierIndices[i]];
			auto bbox = bezierEntity.bezier.boundingBoxConvexhull();
			bbox.expand(bezierEntity.radius);
			aabb.expand(bbox);
		}
		Vec3 size = aabb.size();

		struct SahContext {
			AABB aabb_L;
			AABB aabb_R;
			std::vector<int> triangles_L;
			std::vector<int> triangles_R;

			void clear() {
				aabb_L = AABB();
				aabb_R = AABB();
				triangles_L.clear();
				triangles_R.clear();
			}

			// 分割した場合のコスト期待値
			double cost(double parentArea) const {
				return 2.0 * kCOST_INTERSECT_AABB
					+ (aabb_L.surface_area() / parentArea) * triangles_L.size() * kCOST_INTERSECT_TRIANGLE +
					+(aabb_R.surface_area() / parentArea) * triangles_R.size() * kCOST_INTERSECT_TRIANGLE;
			}
		};
		double parentArea = aabb.surface_area();
		SahContext sah_context;
		double sep_cost = std::numeric_limits<double>::max();

		SahContext sah_new_context;
		int tryCount = (int)std::max(/* calc */kBVH_SAH_POWER * log(bezierIndices.size()), 5.0);
		// int tryCount = 5;
		for (int sah_i = 0; sah_i < tryCount; ++sah_i) {
			sah_new_context.clear();

			auto dim = random->generate() % 3;

			// 中央の重点サンプリング
			double border;
			do {
				border = aabb.center()[dim] + size[dim] * 0.5 * 0.2 * koku(3.0, random);
			} while (border < aabb.min_position[dim] || aabb.max_position[dim] < border);

			// ボーダーに基づいて振り分ける
			for (std::size_t i = 0; i < bezierIndices.size(); ++i) {
				auto bezierIndex = bezierIndices[i];

				//double value0 = vertices[triangle.index[0]][dim];
				//double value1 = vertices[triangle.index[1]][dim];
				//double value2 = vertices[triangle.index[2]][dim];
				
				std::array<Vec3, 4> convexhull = beziers[bezierIndex].bezier.convexhull();

				std::array<double, 4> values = {
					convexhull[0][dim],
					convexhull[1][dim],
					convexhull[2][dim],
					convexhull[3][dim],
				};

				if (std::any_of(values.begin(), values.end(), [border](double value) { return value <= border; })) {
					sah_new_context.triangles_L.push_back(bezierIndex);

					auto bezierEntity = beziers[bezierIndices[i]];
					auto bbox = bezierEntity.bezier.boundingBoxConvexhull();
					bbox.expand(bezierEntity.radius);
					sah_new_context.aabb_L.expand(bbox);
				}
				if (std::any_of(values.begin(), values.end(), [border](double value) { return border < value; })) {
					sah_new_context.triangles_R.push_back(bezierIndex);

					auto bezierEntity = beziers[bezierIndices[i]];
					auto bbox = bezierEntity.bezier.boundingBoxConvexhull();
					bbox.expand(bezierEntity.radius);
					sah_new_context.aabb_R.expand(bbox);
				}
				//if (value0 <= border || value1 <= border || value2 <= border) {
				//	sah_new_context.triangles_L.push_back(triangle);
				//	for (int j = 0; j < 3; ++j) {
				//		sah_new_context.aabb_L.expand(vertices[triangle.index[j]]);
				//	}
				//}
				//if (border < value0 || border < value1 || border < value2) {
				//	sah_new_context.triangles_R.push_back(triangle);
				//	for (int j = 0; j < 3; ++j) {
				//		sah_new_context.aabb_R.expand(vertices[triangle.index[j]]);
				//	}
				//}
			}

			auto this_cost = sah_new_context.cost(parentArea);
			if (this_cost < sep_cost) {
				sep_cost = this_cost;
				sah_context = sah_new_context;
			}
		}

		// 分割しなかった場合のコスト
		double no_sep_cost = bezierIndices.size() * kCOST_INTERSECT_TRIANGLE;

		if (no_sep_cost < sep_cost) {
			BVHLeaf leaf;
			leaf.bezierIndices = bezierIndices;
			return leaf;
		}

		if (sah_context.triangles_L.size() == bezierIndices.size() ||
			sah_context.triangles_R.size() == bezierIndices.size()) {
			printf("warning! \n");
			BVHLeaf leaf;
			leaf.bezierIndices = bezierIndices;
			return leaf;
		}

		// printf("depth [%d], sep %d => (%d, %d) \n", depth, (int)bezierIndices.size(), (int)sah_context.triangles_L.size(), (int)sah_context.triangles_R.size());

		std::unique_ptr<BVHBranch> branch(new BVHBranch());
		branch->aabb_L = sah_context.aabb_L;
		branch->aabb_R = sah_context.aabb_R;
		//branch->lhs = build_tree(sah_context.triangles_L, vertices, random);
		//branch->rhs = build_tree(sah_context.triangles_R, vertices, random);

		if (depth < 5) {
			tbb::task_group g;
			uint32_t LSeed = random->generate();
			uint32_t RSeed = random->generate();
			
			g.run([&branch, &sah_context, beziers, LSeed, depth]() {
				Xor childRandomL(LSeed);
				branch->lhs = build_tree(sah_context.triangles_L, beziers, &childRandomL, depth + 1);
			});
			g.run([&branch, &sah_context, beziers, RSeed, depth]() {
				Xor childRandomR(RSeed);
				branch->rhs = build_tree(sah_context.triangles_R, beziers, &childRandomR, depth + 1);
			});
			g.wait();
		}
		else {
			branch->lhs = build_tree(sah_context.triangles_L, beziers, random);
			branch->rhs = build_tree(sah_context.triangles_R, beziers, random);
		}

		return branch;
	}

	// TODO Name
	struct BVHBezierIntersection : public CurveIntersection {
		BVHBezierIntersection(){}
		BVHBezierIntersection(const CurveIntersection& intersection, Vec3 tangent_, int index) :CurveIntersection(intersection), bezierIndex(index), tangent(tangent_){
		}
		int bezierIndex = -1;
		Vec3 tangent;
	};

	// 最も基本のBVH衝突判定
	inline bool intersect_bvh(const Ray &ray, const Vec3 &inv_d, const MoveAndRotate &projection, const BVHNode &node, const BezierEntity *beziers, BVHBezierIntersection *intersection, double *tmin) {
		bool intersected = false;
		if (node.is<BVHLeaf>()) {
			const BVHLeaf &leaf = node.get<BVHLeaf>();
			for (int i = 0; i < leaf.bezierIndices.size(); ++i) {
				auto index = leaf.bezierIndices[i];
				auto bezier = beziers[index].bezier.transform(projection);
				auto radius = beziers[index].radius;
				rt::CurveIntersection thisIntersection;
				bool isect = rt::intersect_bezier(4, radius, radius * radius, bezier, bezier, 0.0, 1.0, tmin, &thisIntersection);
				Vec3 tangent;
				if (isect) {
					tangent = bezier.tangent(thisIntersection.bezier_t);
					auto p = bezier.evaluate(thisIntersection.bezier_t);
					if (rt::distanceSqPointRay(rt::Vec3(0.0), p, tangent) < radius * radius) {
						isect = false;
					}
				}
				if (isect) {
					*intersection = BVHBezierIntersection(thisIntersection, glm::normalize(tangent), index);
					intersected = true;
				}
			}
		}
		else {
			const std::unique_ptr<BVHBranch> &branch = node.get<std::unique_ptr<BVHBranch>>();
			if (auto aabb_tmin = intersect_aabb(ray, inv_d, branch->aabb_L)) {
				if (intersect_bvh(ray, inv_d, projection, branch->lhs, beziers, intersection, tmin)) {
					intersected = true;
				}
			}

			if (auto aabb_tmin = intersect_aabb(ray, inv_d, branch->aabb_R)) {
				if (intersect_bvh(ray, inv_d, projection, branch->rhs, beziers, intersection, tmin)) {
					intersected = true;
				}
			}
		}
		return intersected;
	}

	//// QBVH
	//inline bool intersect_bvh(const Ray &ray, const Vec3 &inv_d, const QBVHNode &node, const Vec3 *vertices, BVHBezierIntersection *intersection, double *tmin) {
	//	bool intersected = false;

	//	if (node.is<BVHLeaf>()) {
	//		const BVHLeaf &leaf = node.get<BVHLeaf>();
	//		for (int i = 0; i < leaf.triangles.size(); ++i) {
	//			const Vec3 &v0 = vertices[leaf.triangles[i].index[0]];
	//			const Vec3 &v1 = vertices[leaf.triangles[i].index[1]];
	//			const Vec3 &v2 = vertices[leaf.triangles[i].index[2]];
	//			TriangleIntersection thisIntersection;
	//			if (intersect_triangle(ray, v0, v1, v2, &thisIntersection, tmin)) {
	//				*intersection = BVHBezierIntersection(thisIntersection, i);
	//				intersected = true;
	//			}
	//		}
	//	} else if(node.is<std::unique_ptr<QBVHBranchx2>>()) {
	//		const std::unique_ptr<QBVHBranchx2> &branch = node.get<std::unique_ptr<QBVHBranchx2>>();
	//		AABBIntersection_x4 intersections = intersect_aabb_4x_simd(ray, inv_d, branch->aabb_x4);

	//		for (int i = 0; i < 2; ++i) {
	//			if (intersections.tmins[i] && *intersections.tmins[i] < *tmin) {
	//				if (intersect_bvh(ray, inv_d, branch->nodes[i], vertices, intersection, tmin)) {
	//					intersected = true;
	//				}

	//				//auto aabb = branch->aabb_x4._aabbs[i];
	//				//auto s = aabb.size();
	//				//auto c = aabb.center();
	//				//ofSetColor(ofColor::blue);
	//				//ofNoFill();
	//				//ofDrawBox(c.x, c.y, c.z, s.x, s.y, s.z);
	//				//ofFill();
	//			}
	//		}
	//	}
	//	else {
	//		const std::unique_ptr<QBVHBranchx4> &branch = node.get<std::unique_ptr<QBVHBranchx4>>();

	//		AABBIntersection_x4 intersections = intersect_aabb_4x_simd(ray, inv_d, branch->aabb_x4);

	//		for (int i = 0; i < 4; ++i) {
	//			if (intersections.tmins[i] && *intersections.tmins[i] < *tmin) {
	//				if (intersect_bvh(ray, inv_d, branch->nodes[i], vertices, intersection, tmin)) {
	//					intersected = true;
	//				}

	//				//auto aabb = branch->aabb_x4._aabbs[i];
	//				//auto s = aabb.size();
	//				//auto c = aabb.center();
	//				//ofSetColor(ofColor::blue);
	//				//ofNoFill();
	//				//ofDrawBox(c.x, c.y, c.z, s.x, s.y, s.z);
	//				//ofFill();
	//			}
	//		}
	//	}
	//	return intersected;
	//}


	class BezierBVH {
	public:
		BezierBVH(const std::vector<BezierEntity> &beziers):_beziers(beziers) {
			std::vector<int> bezierIndices;
			for (int i = 0; i < _beziers.size(); ++i) {
				bezierIndices.push_back(i);
			}
			_node = build_tree(bezierIndices, _beziers.data(), &_random);
			// _qnode = to_qbvh(_node);
			// _flatten = flatten_bvh(_node);

			for (int i = 0; i < bezierIndices.size(); ++i) {
				auto bezierEntity = beziers[bezierIndices[i]];
				auto bbox = bezierEntity.bezier.boundingBoxConvexhull();
				bbox.expand(bezierEntity.radius);
				_aabb.expand(bbox);
			}
		}
		bool intersect(const Ray &ray, BVHBezierIntersection *intersection, double *tmin) const {
			if (glm::all(glm::isfinite(ray.d) && glm::isfinite(ray.o)) == false) {
				return false;
			}
			Vec3 inv_d = Vec3(1.0) / ray.d;
			if (auto aabb_tmin = intersect_aabb(ray, inv_d, _aabb)) {
				auto projection = rt::ray_projection(ray.o, ray.d);

				// Simple
				return intersect_bvh(ray, inv_d, projection, _node, _beziers.data(), intersection, tmin);

				// Flat
				// return intersect_bvh(ray, inv_d, _flatten, 1, _vertices.data(), intersection, tmin);

				// QBVH
				// return intersect_bvh(ray, inv_d, _qnode, _vertices.data(), intersection, tmin);
			}
			return false;
		}
		const BezierEntity &bezier(int index) const {
			return _beziers[index];
		}

		AABB _aabb;
		std::vector<BezierEntity> _beziers;
		BVHNode _node;
		Xor _random;

		// QBVHNode _qnode;
		// std::vector<BVHFlattenNode> _flatten;
	};
}


/*
	このあたりあんまり速度が上がらなかった系

	struct BVHFlattenBranch {
		AABB aabb_L;
		AABB aabb_R;
	};
	using BVHFlattenNode = mapbox::util::variant<BVHLeaf, BVHFlattenBranch>;

	inline int left_child(int index) {
		return index << 1;
	}
	inline int right_child(int index) {
		return (index << 1) + 1;
	}
	inline int parent(int index) {
		return index >> 1;
	}
	inline int depth_count(const BVHNode &bvhNode, int depth = 1) {
		if (bvhNode.is<BVHLeaf>()) {
			return depth;
		}
		else {
			const std::unique_ptr<BVHBranch> &branch = bvhNode.get<std::unique_ptr<BVHBranch>>();
			return std::max(depth_count(branch->lhs, depth + 1), depth_count(branch->rhs, depth + 1));
		}
	}

	inline int binary_storage_size(const BVHNode &bvhNode) {
		int depth = depth_count(bvhNode);
		int nodes = 1;
		int size = 0;
		for (int i = 0; i < depth; ++i) {
			size += nodes;
			nodes = nodes * 2;
		}
		return size + 1;
	}

	inline void flatten(const BVHNode &bvhNode, std::vector<BVHFlattenNode> &storage, int index) {
		if (bvhNode.is<BVHLeaf>()) {
			storage[index] = bvhNode.get<BVHLeaf>();
		}
		else {
			const std::unique_ptr<BVHBranch> &branch = bvhNode.get<std::unique_ptr<BVHBranch>>();

			BVHFlattenBranch flatbranch;
			flatbranch.aabb_L = branch->aabb_L;
			flatbranch.aabb_R = branch->aabb_R;
			storage[index] = flatbranch;

			flatten(branch->lhs, storage, left_child(index));
			flatten(branch->rhs, storage, right_child(index));
		}
	}
	inline std::vector<BVHFlattenNode> flatten_bvh(const BVHNode &bvhNode) {
		std::vector<BVHFlattenNode> nodes(binary_storage_size(bvhNode));
		flatten(bvhNode, nodes, 1);
		return nodes;
	}
	// flat bvh
	inline bool intersect_bvh(const Ray &ray, const Vec3 &inv_d, const std::vector<BVHFlattenNode> &storage, int index, const Vec3 *vertices, BVHBezierIntersection *intersection, double *tmin) {
		bool intersected = false;
		const BVHFlattenNode &node = storage[index];
		if (node.is<BVHLeaf>()) {
			const BVHLeaf &leaf = node.get<BVHLeaf>();
			for (int i = 0; i < leaf.triangles.size(); ++i) {
				const Vec3 &v0 = vertices[leaf.triangles[i].index[0]];
				const Vec3 &v1 = vertices[leaf.triangles[i].index[1]];
				const Vec3 &v2 = vertices[leaf.triangles[i].index[2]];
				TriangleIntersection thisIntersection;
				if (intersect_triangle(ray, v0, v1, v2, &thisIntersection, tmin)) {
					*intersection = BVHBezierIntersection(thisIntersection, i);
					intersected = true;
				}
			}
		}
		else {
			const BVHFlattenBranch &branch = node.get<BVHFlattenBranch>();
			if (auto aabb_tmin = intersect_aabb(ray, inv_d, branch.aabb_L)) {
				if (intersect_bvh(ray, inv_d, storage, left_child(index), vertices, intersection, tmin)) {
					intersected = true;
				}
			}

			if (auto aabb_tmin = intersect_aabb(ray, inv_d, branch.aabb_R)) {
				if (intersect_bvh(ray, inv_d, storage, right_child(index), vertices, intersection, tmin)) {
					intersected = true;
				}
			}
		}
		return intersected;
	}
*/