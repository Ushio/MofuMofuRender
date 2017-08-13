#pragma once

#include "optional.hpp"
#include "geometry.hpp"

namespace rt {
	struct AABB {
		Vec3 min_position = Vec3(std::numeric_limits<double>::max());
		Vec3 max_position = Vec3(-std::numeric_limits<double>::max());
		AABB() {}
		AABB(const Vec3 &min_p, const Vec3 &max_p) :min_position(min_p), max_position(max_p) {
			for (int i = 0; i < 3; ++i) {
				if (max_position[i] < min_position[i]) {
					std::swap(max_position[i], min_position[i]);
				}
			}
		}

		void expand(const AABB &aabb) {
			min_position = glm::min(min_position, aabb.min_position);
			max_position = glm::max(max_position, aabb.max_position);
		}
		void expand(const Vec3 &p) {
			min_position = glm::min(min_position, p);
			max_position = glm::max(max_position, p);
		}
		void expand(double dxyz) {
			min_position -= Vec3(dxyz);
			max_position += Vec3(dxyz);
		}
		bool contains(const Vec3 &p) const {
			for (int dim = 0; dim < 3; ++dim) {
				if (p[dim] < min_position[dim] || max_position[dim] < p[dim]) {
					return false;
				}
			}
			return true;
		}
		bool empty() const {
			return glm::any(glm::greaterThan(min_position, max_position));
		}
		Vec3 size() const {
			return glm::max(max_position - min_position, Vec3());
		}
		Vec3 center() const {
			return (max_position + min_position) * 0.5;
		}

		double surface_area() const {
			Vec3 s = size();
			return (s.x * s.z + s.x * s.y + s.z * s.y) * 2.0;
		}
	};

	// 分岐なし実装
	// https://tavianator.com/fast-branchless-raybounding-box-intersections/
	inline std::experimental::optional<float> intersect_aabb(const Ray &ray, const rt::Vec3 &inv_d, const AABB& aabb) {
		double tx1 = (aabb.min_position.x - ray.o.x) * inv_d.x;
		double tx2 = (aabb.max_position.x - ray.o.x) * inv_d.x;
		double tmin = glm::min(tx1, tx2);
		double tmax = glm::max(tx1, tx2);

		double ty1 = (aabb.min_position.y - ray.o.y) * inv_d.y;
		double ty2 = (aabb.max_position.y - ray.o.y) * inv_d.y;
		tmin = glm::max(tmin, glm::min(ty1, ty2));
		tmax = glm::min(tmax, glm::max(ty1, ty2));

		double tz1 = (aabb.min_position.z - ray.o.z) * inv_d.z;
		double tz2 = (aabb.max_position.z - ray.o.z) * inv_d.z;
		tmin = glm::max(tmin, glm::min(tz1, tz2));
		tmax = glm::min(tmax, glm::max(tz1, tz2));

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
		return std::experimental::optional<float>();
	}

	struct AABB2d {
		Vec2 min_position = Vec2(std::numeric_limits<double>::max());
		Vec2 max_position = Vec2(-std::numeric_limits<double>::max());
		AABB2d() {}
		AABB2d(const Vec2 &min_p, const Vec2 &max_p) :min_position(min_p), max_position(max_p) {}

		void expand(const Vec2 &p) {
			min_position = glm::min(min_position, p);
			max_position = glm::max(max_position, p);
		}
		void expand(double dxyz) {
			min_position -= Vec2(dxyz);
			max_position += Vec2(dxyz);
		}
		bool contains(const Vec2 &p) const {
			for (int dim = 0; dim < 2; ++dim) {
				if (p[dim] < min_position[dim] || max_position[dim] < p[dim]) {
					return false;
				}
			}
			return true;
		}
		bool empty() const {
			return glm::any(glm::greaterThan(min_position, max_position));
		}
		Vec2 size() const {
			return glm::max(max_position - min_position, Vec2());
		}
		Vec2 center() const {
			return (max_position + min_position) * 0.5;
		}
	};
}