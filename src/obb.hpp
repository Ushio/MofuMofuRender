#pragma once

#include <optional.hpp>
#include "geometry.hpp"

namespace rt {

	struct OBB {
		// 中心
		Vec3 ac;

		// 軸
		std::array<Vec3, 3> a;

		// サイズの幅の半分
		Vec3 h;

		void expand(double dxyz) {
			h += Vec3(dxyz);
		}

		double surface_area() const {
			Vec3 s = h * 2.0;
			return (s.x * s.z + s.x * s.y + s.z * s.y) * 2.0;
		}
	};

	inline std::experimental::optional<double> intersect_obb(const Ray &ray, const OBB &obb) {
		auto P = obb.ac - ray.o;
		double e0 = glm::dot(P, obb.a[0]);
		double one_over_cosTheta0 = 1.0 / glm::dot(ray.d, obb.a[0]);
		double tmin0 = (e0 - obb.h[0]) * one_over_cosTheta0;
		double tmax0 = (e0 + obb.h[0]) * one_over_cosTheta0;
		if (tmax0 < tmin0) { std::swap(tmax0, tmin0); }

		double e1 = glm::dot(P, obb.a[1]);
		double one_over_cosTheta1 = 1.0 / glm::dot(ray.d, obb.a[1]);
		double tmin1 = (e1 - obb.h[1]) * one_over_cosTheta1;
		double tmax1 = (e1 + obb.h[1]) * one_over_cosTheta1;
		if (tmax1 < tmin1) { std::swap(tmax1, tmin1); }

		double e2 = glm::dot(P, obb.a[2]);
		double one_over_cosTheta2 = 1.0 / glm::dot(ray.d, obb.a[2]);
		double tmin2 = (e2 - obb.h[2]) * one_over_cosTheta2;
		double tmax2 = (e2 + obb.h[2]) * one_over_cosTheta2;
		if (tmax2 < tmin2) { std::swap(tmax2, tmin2); }

		double tmin = glm::max(glm::max(tmin0, tmin1), tmin2);
		double tmax = glm::min(glm::min(tmax0, tmax1), tmax2);

		// tmin <= tmax は直線が交点を持っている
		// tmin < 0.0
		//    :: tmaxが交点
		// 0.0 < tmin
		//    :: tminが交点
		// tmax < 0.0
		//    :: 当たっていない

		bool intersect = tmin <= tmax && 0.0 < tmax;
		if (intersect) {
			return tmin < 0.0 ? tmax : tmin;
		}
		return std::experimental::optional<double>();
	}

	class OnlineOBB {
	public:
		OnlineOBB(const std::array<Vec3, 3> &axis) :_axis(axis) {

		}

		void addSample(const Vec3 &p) {
			for (int i = 0; i < 3; ++i) {
				double projected = glm::dot(_axis[i], p);
				_min[i] = glm::min(_min[i], projected);
				_max[i] = glm::max(_max[i], projected);
			}
		}

		// p に加えて、expandRadius分も広げる
		void addSample(const Vec3 &p, double expandRadius) {
			for (int i = 0; i < 3; ++i) {
				double projected = glm::dot(_axis[i], p);
				_min[i] = glm::min(_min[i], projected - expandRadius);
				_max[i] = glm::max(_max[i], projected + expandRadius);
			}
		}
		OBB obb() const {
			Vec3 ac_local = (_max + _min) * 0.5;

			OBB o;
			o.ac = _axis[0] * ac_local[0] + _axis[1] * ac_local[1] + _axis[2] * ac_local[2];
			o.a = _axis;
			o.h = (_max - _min) * 0.5;
			return o;
		}

		std::array<Vec3, 3> _axis;
		Vec3 _min = Vec3(std::numeric_limits<double>::max());
		Vec3 _max = Vec3(-std::numeric_limits<double>::max());
	};
}