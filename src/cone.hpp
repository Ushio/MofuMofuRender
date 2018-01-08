#pragma once

#include "geometry.hpp"
#include "misc.hpp"

namespace rt {
	struct Cone {
		double h = 1.0;
		double r = 1.0;
		double min_h = 0.0;
		double max_h = 1.0;
	};

	inline bool intersect_cone(Vec3 o, Vec3 d, Cone cone, double *tmin) {
		double h = cone.h;
		double r = cone.r;

		double k = Sqr(h) / Sqr(r);

		double a = k * (d.x * d.x + d.z * d.z) - d.y * d.y;
		double b = 2.0 * k * (o.x * d.x + o.z * d.z) - 2.0 * (o.y - h) * d.y;
		double c = k * (o.x * o.x + o.z * o.z) - Sqr(o.y - h);

		double D = b * b - 4.0 * a * c;
		if (D < 0.0) {
			return false;
		}

		double t;
		bool isect = false;

		double sqrt_D = std::sqrt(D);

		t = (-b + sqrt_D) / (2.0 * a);
		if (0.0 < t && t < *tmin) {
			double y = o.y + d.y * t;
			if (cone.min_h <= y && y <= cone.max_h) {
				*tmin = t;
				isect = true;
			}
		}

		t = (-b - sqrt_D) / (2.0 * a);
		if (0.0 < t && t < *tmin) {
			double y = o.y + d.y * t;
			if (cone.min_h <= y && y <= cone.max_h) {
				*tmin = t;
				isect = true;
			}
		}

		return isect;
	}

	// 円錐の方程式の、(∂u/∂x, ∂u/∂y, ∂u/∂z)
	inline Vec3 gradient_cone(Vec3 p, Cone cone) {
		double hh_over_rr = cone.h * cone.h / (cone.r * cone.r);
		double dudx = 2.0 * hh_over_rr * p.x;
		double dudy = 2.0 * (cone.h - p.y);
		double dudz = 2.0 * hh_over_rr * p.z;
		return Vec3(dudx, dudy, dudz);
	}

	struct Cylinder {
		double r = 1.0;
		double min_h = 0.0;
		double max_h = 1.0;
	};
	inline bool intersect_cylinder(Vec3 o, Vec3 d, Cylinder cylinder, double *tmin) {
		double r = cylinder.r;

		double a = d.x * d.x + d.z * d.z;
		double b = 2.0 * (o.x * d.x + o.z * d.z);
		double c = o.x * o.x + o.z * o.z - r * r;

		double D = b * b - 4.0 * a * c;
		if (D < 0.0) {
			return false;
		}

		double t;
		bool isect = false;

		t = (-b + sqrt(D)) / (2.0 * a);
		if (0.0 < t && t < *tmin) {
			double y = o.y + d.y * t;
			if (cylinder.min_h <= y && y <= cylinder.max_h) {
				*tmin = t;
				isect = true;
			}
		}

		t = (-b - sqrt(D)) / (2.0 * a);
		if (0.0 < t && t < *tmin) {
			double y = o.y + d.y * t;
			if (cylinder.min_h <= y && y <= cylinder.max_h) {
				*tmin = t;
				isect = true;
			}
		}

		return isect;
	}


	struct TruncatedCone {
		Vec3 p;
		double p_radius = 0.0;
		Vec3 q;
		double q_radius = 1.0;
	};
	inline bool intersect_cone(Vec3 o, Vec3 d, TruncatedCone cone, double *tmin, Vec3 *n) {
		double L = glm::distance(cone.p, cone.q);
		if (L <= glm::epsilon<double>()) {
			return false;
		}

		if (cone.q_radius < cone.p_radius) {
			std::swap(cone.p, cone.q);
			std::swap(cone.p_radius, cone.q_radius);
		}

		// p is upper
		Vec3 cone_o = cone.q;
		Vec3 cone_d = (cone.p - cone.q) / L;

		// 円錐を水平に戻す
		// ここはライブラリによってはcone_d.y < -0.999...のとき、未定義になる可能性あり
		Quat r_to_cone_local = glm::rotation(cone_d, Vec3(0.0, 1.0, 0.0));
		Vec3 t_to_cone_local = -cone_o;
		Vec3 d_cone_local = r_to_cone_local * d;
		Vec3 o_cone_local = r_to_cone_local * (o + t_to_cone_local);

		if (std::fabs(cone.q_radius - cone.p_radius) < 0.0000001) {
			// intersect as cylinder
			Cylinder cylinder;
			cylinder.r = (cone.q_radius + cone.p_radius) * 0.5f;
			cylinder.min_h = 0.0;
			cylinder.max_h = L;
			if (intersect_cylinder(o_cone_local, d_cone_local, cylinder, tmin)) {
				Vec3 p = o_cone_local + d_cone_local * (*tmin);
				*n = glm::normalize(Vec3(p.x, 0.0, p.z));
				if (0.0 < glm::dot(*n, d_cone_local)) {
					*n = -*n;
				}
				*n = glm::inverse(r_to_cone_local) * (*n);
				return true;
			}
			else {
				return false;
			}
		}

		// intersect as cone

		Cone pure_cone;
		pure_cone.h = cone.q_radius / (cone.q_radius - cone.p_radius) * L;
		pure_cone.r = cone.q_radius;
		pure_cone.min_h = 0.0;
		pure_cone.max_h = L;
		if (intersect_cone(o_cone_local, d_cone_local, pure_cone, tmin)) {
			Vec3 p = o_cone_local + d_cone_local * (*tmin);

			/*
			三角錐の方程式 ~~~~ = 0 において、
			(∂u/∂x)⊿x + (∂u/∂y)⊿y + (∂u/∂z)⊿z == 0
			となるのは、(⊿x, ⊿y, ⊿z) の移動が三角錐の表面上である場合のみである。
			その移動ベクトルは面をなし、その移動ベクトルと(∂u/∂x, ∂u/∂y, ∂u/∂z)の内積がゼロである。
			これはつまり、(∂u/∂x, ∂u/∂y, ∂u/∂z)は表面と直交している。よって、これは法線ベクトルとなる。
			*/
			*n = glm::normalize(gradient_cone(p, pure_cone));
			if (0.0 < glm::dot(*n, d_cone_local)) {
				*n = -*n;
			}
			*n = glm::inverse(r_to_cone_local) * (*n);
			return true;
		}
		return false;
	}
}