#pragma once

#include <vector>
#include <immintrin.h>
#include <mapbox/variant.hpp>
#include <optional.hpp>

#include "geometry.hpp"
#include "peseudo_random.hpp"
#include "aabb.hpp"
#include "obb.hpp"
#include "bezier.hpp"
#include "misc.hpp"
#include "stats.hpp"

#include <tbb/tbb.h>

namespace rt {
	// static double kBVH_SAH_POWER = 3.0;
	static double kBVH_SAH_POWER = 30.0;

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

	static const double kCOST_INTERSECT_AABB = 1.0;
	static const double kCOST_INTERSECT_TRIANGLE = 4.0;

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

	inline BVHNode build_tree(const std::vector<int> &bezierIndices, const BezierEntity *beziers, PeseudoRandom *random, int depth) {
		//auto write_leaf = [&](std::vector<int> indices) {
		//	fprintf(fp, "leaf\n");
		//	for (int index : indices) {
		//		const auto &bezier = beziers[index];
		//		fprintf(fp, "%.10f/%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n", bezier.radius,
		//			bezier.bezier[0].x, bezier.bezier[0].y, bezier.bezier[0].z,
		//			bezier.bezier[1].x, bezier.bezier[1].y, bezier.bezier[1].z,
		//			bezier.bezier[2].x, bezier.bezier[2].y, bezier.bezier[2].z);
		//	}
		//};
		
		int maxTriangles = 1;
		if (bezierIndices.size() <= maxTriangles) {
			BVHLeaf leaf;
			leaf.bezierIndices = bezierIndices;
			// write_leaf(bezierIndices);
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

				double radius = beziers[bezierIndex].radius;
				std::array<Vec3, 4> convexhull = beziers[bezierIndex].bezier.convexhull();

				std::array<double, 4> values = {
					convexhull[0][dim],
					convexhull[1][dim],
					convexhull[2][dim],
					convexhull[3][dim],
				};

				if (std::any_of(values.begin(), values.end(), [border, radius](double value) { return value - radius <= border; })) {
					sah_new_context.triangles_L.push_back(bezierIndex);

					auto bezierEntity = beziers[bezierIndices[i]];
					auto bbox = bezierEntity.bezier.boundingBoxConvexhull();
					bbox.expand(bezierEntity.radius);
					sah_new_context.aabb_L.expand(bbox);
				}
				if (std::any_of(values.begin(), values.end(), [border, radius](double value) { return border < value + radius; })) {
					sah_new_context.triangles_R.push_back(bezierIndex);

					auto bezierEntity = beziers[bezierIndices[i]];
					auto bbox = bezierEntity.bezier.boundingBoxConvexhull();
					bbox.expand(bezierEntity.radius);
					sah_new_context.aabb_R.expand(bbox);
				}
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
			// write_leaf(bezierIndices);
			return leaf;
		}

		if (sah_context.triangles_L.size() == bezierIndices.size() ||
			sah_context.triangles_R.size() == bezierIndices.size()) {
			printf("warning! \n");
			BVHLeaf leaf;
			leaf.bezierIndices = bezierIndices;
			// write_leaf(bezierIndices);
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
			branch->lhs = build_tree(sah_context.triangles_L, beziers, random, depth + 1);
			branch->rhs = build_tree(sah_context.triangles_R, beziers, random, depth + 1);
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
	inline bool intersect_bvh(const Ray &ray, const Vec3 &inv_d, const MoveAndRotate &projection, const BVHNode &node, const BezierEntity *beziers, const double *maxDistanceSqsFromSegment, BVHBezierIntersection *intersection, double *tmin) {
		bool intersected = false;
		if (node.is<BVHLeaf>()) {
			const BVHLeaf &leaf = node.get<BVHLeaf>();
			for (int i = 0; i < leaf.bezierIndices.size(); ++i) {
				auto index = leaf.bezierIndices[i];

				// 直線からの最大距離よりも離れている場合は、衝突はありえないので、早期に枝刈りをする
				// AABB による判定はかなりざっくりなので、とても効果がある
				const BezierQuadratic3D &origBezier = beziers[index].bezier;
				if (maxDistanceSqsFromSegment[index] < distanceSqRayRay(ray.o, ray.d, origBezier[0], origBezier[2] - origBezier[0])) {
					continue;
				}

				BezierQuadratic3D bezier = beziers[index].bezier.transform(projection);

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
		} else {
			const std::unique_ptr<BVHBranch> &branch = node.get<std::unique_ptr<BVHBranch>>();
			if (auto aabb_tmin = intersect_aabb(ray, inv_d, branch->aabb_L)) {
				if (intersect_bvh(ray, inv_d, projection, branch->lhs, beziers, maxDistanceSqsFromSegment, intersection, tmin)) {
					intersected = true;
				}
			}

			if (auto aabb_tmin = intersect_aabb(ray, inv_d, branch->aabb_R)) {
				if (intersect_bvh(ray, inv_d, projection, branch->rhs, beziers, maxDistanceSqsFromSegment, intersection, tmin)) {
					intersected = true;
				}
			}
		}
		return intersected;
	}
	
	class BezierBVH {
	public:
		BezierBVH(const std::vector<BezierEntity> &beziers):_beziers(beziers) {
			std::vector<int> bezierIndices;
			for (int i = 0; i < _beziers.size(); ++i) {
				bezierIndices.push_back(i);
			}
			_node = build_tree(bezierIndices, _beziers.data(), &_random, 0);

			for (int i = 0; i < bezierIndices.size(); ++i) {
				auto bezierEntity = beziers[bezierIndices[i]];
				auto bbox = bezierEntity.bezier.boundingBoxConvexhull();
				bbox.expand(bezierEntity.radius);
				_aabb.expand(bbox);
			}

			// ブリュートフォースのタイトフィッティング
			_maxDistanceSqsFromSegment.resize(_beziers.size());
			for (int i = 0; i < _beziers.size(); ++i) {
				auto bezier = _beziers[i];

				double maxDistanceSq = 0.0;
				const int NSplit = 1000;
				for (int j = 0; j < NSplit; ++j) {
					static Remap toT(0, NSplit - 1, 0.0, 1.0);
					double t = toT(j);
					auto p = bezier.bezier.evaluate(t);
					double distanceSq = rt::distanceSqPointRay(p, bezier.bezier[0], bezier.bezier[2] - bezier.bezier[0]);
					double d = glm::sqrt(distanceSq);
					double maxDistance = (d + bezier.radius);
					maxDistanceSq = std::max(maxDistanceSq, maxDistance * maxDistance);
				}
				_maxDistanceSqsFromSegment[i] = maxDistanceSq;
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
				return intersect_bvh(ray, inv_d, projection, _node, _beziers.data(), _maxDistanceSqsFromSegment.data(), intersection, tmin);
			}
			return false;
		}
		const BezierEntity &bezier(int index) const {
			return _beziers[index];
		}

		AABB _aabb;
		std::vector<BezierEntity> _beziers;

		// 早期枝刈りのための、線分からの最大距離の平方を前計算する
		std::vector<double> _maxDistanceSqsFromSegment;
		
		BVHNode _node;
		Xor _random;
	};
}
