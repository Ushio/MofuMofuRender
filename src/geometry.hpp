#pragma once

#include <glm/glm.hpp>
#include <glm/ext.hpp>

namespace rt {
	typedef glm::dvec2 Vec2;
	typedef glm::dvec3 Vec3;
	typedef glm::dvec4 Vec4;
	typedef glm::dmat3 Mat3;
	typedef glm::dmat4 Mat4;

	struct Ray {
		Ray() {}
		Ray(Vec3 orig, Vec3 dir) :o(orig), d(dir) {}
		Vec3 o;
		Vec3 d;
	};
	inline Vec3 mul3x4(const Mat4 &m, const Vec3 v) {
		return Vec3(m * Vec4(v, 1.0));
	}


	struct Intersection {
		Vec3 normal;
		bool isBack = false;
	};
	struct TriangleIntersection : public Intersection {
		Vec2 uv;
	};

	inline Vec3 triangle_normal(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2, bool isback) {
		Vec3 e1 = v1 - v0;
		Vec3 e2 = v2 - v0;
		return glm::normalize(isback ? glm::cross(e2, e1) : glm::cross(e1, e2));
	}

	inline bool intersect_triangle(const Ray &ray, const Vec3 &v0, const Vec3 &v1, const Vec3 &v2, TriangleIntersection *intersection, double *tmin) {
		Vec3 baryPosition;

		Vec3 e1 = v1 - v0;
		Vec3 e2 = v2 - v0;

		Vec3 p = glm::cross(ray.d, e2);

		double a = glm::dot(e1, p);

		double f = 1.0 / a;

		Vec3 s = ray.o - v0;

		Vec3 q = glm::cross(s, e1);
		baryPosition.z = f * glm::dot(e2, q);
		if (baryPosition.z < 0.0 || *tmin <= baryPosition.z) {
			return false;
		}

		baryPosition.x = f * glm::dot(s, p);
		if (baryPosition.x < 0.0 || baryPosition.x > 1.0) {
			return false;
		}

		baryPosition.y = f * glm::dot(ray.d, q);
		if (baryPosition.y < 0.0 || baryPosition.y + baryPosition.x > 1.0) {
			return false;
		}

		bool isback = a < 0.0;

		*tmin = baryPosition.z;
		intersection->normal = triangle_normal(v0, v1, v2, isback);
		intersection->uv = Vec2(baryPosition.x, baryPosition.y);
		intersection->isBack = isback;

		//*tmin = baryPosition.z;
		//intersection->normal = triangle_normal(v0, v1, v2, false);
		//intersection->uv = Vec2(baryPosition.x, baryPosition.y);
		//intersection->isBack = glm::dot(intersection->normal, ray.d) > 0.0;
		//if (intersection->isBack) {
		//	intersection->normal = -intersection->normal;
		//}

		return true;
	}

	//inline bool intersect_triangle_ref(const Ray &ray, const Vec3 &v0, const Vec3 &v1, const Vec3 &v2, TriangleIntersection *intersection, double *tmin) {
	//	double baryPosition_x;
	//	double baryPosition_y;
	//	double baryPosition_z;

	//	double v0_x = v0.x;
	//	double v0_y = v0.y;
	//	double v0_z = v0.z;

	//	double v1_x = v1.x;
	//	double v1_y = v1.y;
	//	double v1_z = v1.z;

	//	double v2_x = v2.x;
	//	double v2_y = v2.y;
	//	double v2_z = v2.z;

	//	double e1_x = v1_x - v0_x;
	//	double e1_y = v1_y - v0_y;
	//	double e1_z = v1_z - v0_z;

	//	double e2_x = v2_x - v0_x;
	//	double e2_y = v2_y - v0_y;
	//	double e2_z = v2_z - v0_z;

	//	double ray_d_x = ray.d.x;
	//	double ray_d_y = ray.d.y;
	//	double ray_d_z = ray.d.z;

	//	double p_x = ray.d.y * e2_z - e2_y * ray.d.z;
	//	double p_y = ray.d.z * e2_x - e2_z * ray.d.x;
	//	double p_z = ray.d.x * e2_y - e2_x * ray.d.y;

	//	double a = glm::dot(e1, p);

	//	double f = 1.0 / a;

	//	Vec3 s = ray.o - v0;

	//	Vec3 q = glm::cross(s, e1);
	//	baryPosition.z = f * glm::dot(e2, q);
	//	if (baryPosition.z < 0.0 || *tmin <= baryPosition.z) {
	//		return false;
	//	}

	//	baryPosition.x = f * glm::dot(s, p);
	//	if (baryPosition.x < 0.0 || baryPosition.x > 1.0) {
	//		return false;
	//	}

	//	baryPosition.y = f * glm::dot(ray.d, q);
	//	if (baryPosition.y < 0.0 || baryPosition.y + baryPosition.x > 1.0) {
	//		return false;
	//	}

	//	bool isback = a < 0.0;

	//	*tmin = baryPosition.z;
	//	intersection->normal = triangle_normal(v0, v1, v2, isback);
	//	intersection->uv = Vec2(baryPosition.x, baryPosition.y);
	//	intersection->isBack = isback;
	//	return true;
	//}

	inline double triangle_area(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2) {
		auto va = p0 - p1;
		auto vb = p2 - p1;
		return glm::length(glm::cross(va, vb)) * 0.5;
	}


	struct Sphere {
		Vec3 center;
		double radius = 1.0;

		Sphere() {}
		Sphere(const Vec3 &c, double r) :center(c), radius(r) {}
	};

	inline bool intersect_shere(const Ray &ray, const Sphere &s, Intersection *intersection, double *tmin) {
		Vec3 m = ray.o - s.center;
		double b = glm::dot(m, ray.d);
		double radius_squared = s.radius * s.radius;
		double c = glm::dot(m, m) - radius_squared;
		if (c > 0.0 && b > 0.0) {
			return false;
		}
		double discr = b * b - c;
		if (discr < 0.0) {
			return false;
		}

		double sqrt_discr = glm::sqrt(discr);
		double tmin_near = -b - sqrt_discr;
		if (0.0 < tmin_near && tmin_near < *tmin) {
			// back = false
			auto p = ray.o + ray.d * tmin_near;
			intersection->isBack = false;
			*tmin = tmin_near;
			intersection->normal = glm::normalize(p - s.center);
			return true;
		}
		double tmin_far = -b + sqrt_discr;
		if (0.0 < tmin_far && tmin_far < *tmin) {
			// back = true
			auto p = ray.o + ray.d * tmin_far;
			intersection->isBack = true;
			*tmin = tmin_far;
			intersection->normal = glm::normalize(s.center - p);
			return true;
		}
		return false;
	}
}