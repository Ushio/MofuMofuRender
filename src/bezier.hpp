#pragma once

#include <array>
#include <tuple>
#include "geometry.hpp"
#include "aabb.hpp"
#include "misc.hpp"
#include "cardano.hpp"

namespace rt {
	// o + t * d, cとの最近傍距離の平方
	// dは正規化されている必要はない
	inline double distanceSqPointRayT(Vec3 c, Vec3 o, Vec3 d)
	{
		return glm::dot(c - o, d) / glm::dot(d, d);
	}

	// o + d * t と cの最近傍距離の平方を返す
	// dは正規化されている必要はない
	inline double distanceSqPointRay(Vec3 c, Vec3 o, Vec3 d) {
		return glm::length2(glm::cross(c - o, d)) / glm::length2(d);
	}

	/*
	レイとレイの最近傍距離を返す
	レイは正規化されている必要はない
	*/
	inline std::tuple<double, double> closestRayRayST(const rt::Vec3 &p1, const rt::Vec3 &d1, const rt::Vec3 &p2, const rt::Vec3 &d2) {
		Vec3 r = p1 - p2;
		double a = glm::dot(d1, d1);
		double e = glm::dot(d2, d2);
		double f = glm::dot(d2, r);

		double b = glm::dot(d1, d2);
		double denom = a*e - b*b;

		double c = glm::dot(d1, r);
		double s, t;
		if (glm::epsilon<double>() * 100.0 < denom) {
			s = (b*f - c*e) / denom;
		}
		else {
			s = 0.0f;
		}
		t = (b*s + f) / e;
		return std::make_tuple(s, t);
	}

	/*
	レイとレイの最近傍距離を返す
	レイは正規化されている必要はない
	http://www.math.kit.edu/ianm2/lehre/am22016s/media/distance-harvard.pdf
	L(t) = q + t * u
	M(t) = p + t * v
	*/
	inline double distanceRayRay(const rt::Vec3 &q, const rt::Vec3 &u, const rt::Vec3 &p, const rt::Vec3 &v) {
		rt::Vec3 pq = q - p;
		rt::Vec3 uxv = glm::cross(u, v);
		double denom = glm::length(uxv);
		if (denom < glm::epsilon<double>() * 100.0) {
			return glm::sqrt(distanceSqPointRay(q, p, v));
		}
		return glm::length(glm::dot(pq, uxv)) / denom;
	}

	/*
	レイとレイの最近傍距離の平方を返す
	レイは正規化されている必要はない
	http://www.math.kit.edu/ianm2/lehre/am22016s/media/distance-harvard.pdf
	L(t) = q + t * u
	M(t) = p + t * v
	*/
	inline double distanceSqRayRay(const rt::Vec3 &q, const rt::Vec3 &u, const rt::Vec3 &p, const rt::Vec3 &v) {
		rt::Vec3 pq = q - p;
		rt::Vec3 uxv = glm::cross(u, v);
		double denom = glm::length2(uxv);
		if (denom < glm::epsilon<double>() * 100.0) {
			return distanceSqPointRay(q, p, v);
		}
		return glm::length2(glm::dot(pq, uxv)) / denom;
	}

	/*
	レイとレイの最近傍距離を返す
	rt::Vec3 q = Vec3(0, 0, 0);
	rt::Vec3 u = Vec3(0, 0, 1);
	のケース
	レイは正規化されている必要はない
	*/
	inline double distanceRayRay(const rt::Vec3 &p, const rt::Vec3 &v) {
		rt::Vec3 pq = -p;
		rt::Vec3 uxv = rt::Vec3(-v.y, v.x, 0.0);
		double denom2 = uxv.x * uxv.x + uxv.y * uxv.y; // length2
		if (denom2 < glm::epsilon<double>() * 100.0) {
			return glm::sqrt(distanceSqPointRay(Vec3(), p, v));
		}
		double qp_dot_uxv = pq.x * uxv.x + pq.y * uxv.y;
		return glm::sqrt(glm::length2(qp_dot_uxv) / denom2);
	}


	struct MoveAndRotate {
		MoveAndRotate() {}
		MoveAndRotate(const Vec3 &m, const Mat3 &r):move(m), rotation(r) {

		}
		Vec3 operator*(const Vec3 &p) const {
			return rotation * (p + move);
		}
		Vec3 move;
		Mat3 rotation;
	};

	// 向きをすべて Vec3(0.0, 0.0, 1.0) に統一するトランスフォームを返す
	inline MoveAndRotate ray_projection(Vec3 ro, Vec3 rd) {
		Vec3 move = -ro;
		auto d = glm::sqrt(rd.x * rd.x + rd.z * rd.z);
		if (d < glm::epsilon<double>()) {
			Mat3 r;
			// 符号に応じないとだめ
			if (rd.y < 0.0) {
				static Mat3 this_r = glm::rotate(-glm::half_pi<double>(), Vec3(1.0, 0.0, 0.0));
				r = this_r;
			}
			else {
				static Mat3 this_r = glm::rotate(glm::half_pi<double>(), Vec3(1.0, 0.0, 0.0));
				r = this_r;
			}
			return MoveAndRotate(move, r);
		}
		auto over_d = 1.0 / d;
		Mat3 r(
			rd.z * over_d, -rd.x*rd.y * over_d, rd.x,
			0.0, d, rd.y,
			-rd.x * over_d, -rd.y * rd.z * over_d, rd.z
		);
		return MoveAndRotate(move, r);
	}

	inline Vec3 bezier(Vec3 p0, Vec3 cp, Vec3 p1, double t)
	{
		Vec3 mm0 = glm::mix(p0, cp, t);
		Vec3 mm1 = glm::mix(cp, p1, t);
		return glm::mix(mm0, mm1, t);
	}
	inline Vec3 bezier_tangent(Vec3 p0, Vec3 cp, Vec3 p1, double t)
	{
		return 2.0 * t * (p0 - 2.0 * cp + p1) + 2.0 * (cp - p0);
	}

	class BezierQuadratic3D {
	public:
		BezierQuadratic3D() {
		}
		BezierQuadratic3D(Vec3 p0, Vec3 cp, Vec3 p1) :_ps({ p0, cp, p1 }) {
			  
			//_ps[0] = p0;
			//_ps[1] = cp;
			//_ps[2] = p1;
		}
		const Vec3 &operator[](int i) const {
			return _ps[i];
		}
		Vec3 &operator[](int i) {
			return _ps[i];
		}
		Vec3 evaluate(float t) const {
			return bezier(_ps[0], _ps[1], _ps[2], t);
		}
		Vec3 tangent(double t) const {
			return bezier_tangent(_ps[0], _ps[1], _ps[2], t);
		}
		BezierQuadratic3D transform(const MoveAndRotate m) const {
			return BezierQuadratic3D(
				m * _ps[0],
				m * _ps[1],
				m * _ps[2]
			);
		}
		std::array<Vec3, 4> convexhull() const {
			// ただ、フィットが目的の場合は不適切かもしれない。
			// 凸包性を使ったほうがよりタイトにフィット可能
			// http://www.hilano.org/hilano-lab/svg/bezier-interactive-step.html

			Vec3 mm0 = (_ps[0] + _ps[1]) * 0.5;
			Vec3 mm1 = (_ps[1] + _ps[2]) * 0.5;
			return{
				_ps[0],
				mm0,
				mm1,
				_ps[2],
			};
		}
		rt::AABB boundingBox() const {
			rt::AABB aabb(_ps[0], _ps[2]);
			aabb.expand(_ps[1]);
			return aabb;
		}
		rt::AABB boundingBoxConvexhull() const {
			rt::AABB aabb(_ps[0], _ps[2]);
			 aabb.expand((_ps[0] + _ps[1]) * 0.5);
			 aabb.expand((_ps[1] + _ps[2]) * 0.5);
			return aabb;
		}
		std::tuple<BezierQuadratic3D, BezierQuadratic3D> split(double t) const {
			auto pc = evaluate(t);
			auto lhs_cp = glm::mix(_ps[0], _ps[1], t);
			auto rhs_cp = glm::mix(_ps[1], _ps[2], t);

			BezierQuadratic3D lhs(
				_ps[0],
				lhs_cp,
				pc);
			BezierQuadratic3D rhs(
				pc,
				rhs_cp,
				_ps[2]
			);
			return std::make_tuple(lhs, rhs);
		}

		std::array<Vec3, 3> _ps;
	};

	struct CurveIntersection {
		double h = 0.0;
		double bezier_t = 0.0;
	};

	// 曲線の先を尖らせる
	inline bool intersect_bezier(int depth, double radius, double radiusSq, const BezierQuadratic3D &original, const BezierQuadratic3D &c, double v0, double v1, double *tmin, CurveIntersection *intersection)
	{
		// t -> radius係数(0 ~ 1)
		auto radius_f = [](double t) {
			// https://www.desmos.com/calculator/ttwuz1xm7u
			constexpr double a = 1.0;
			constexpr double b = 0.75;
			constexpr double m = 1.0 / (b - a);
			return glm::clamp((t - a) * m, 0.0, 1.0);
		};

		auto b = c.boundingBox();
		double maxR = radius * radius_f(v1);
		if (b.min_position.z >= *tmin || b.max_position.z <= glm::epsilon<double>()
			|| b.min_position.x >= maxR || b.max_position.x <= -maxR
			|| b.min_position.y >= maxR || b.max_position.y <= -maxR) {
			/// the bounding box does not overlap the square
			/// centered at O.
			//ofSetColor(64);
			//drawAABB(b);
			return false;
		}
		else if (depth == 0) {
			/// the maximum recursion depth is reached.
			auto dir = c[2] - c[0];

			// compute w on the line segment.
			double w = dir.x * dir.x + dir.y * dir.y;
			if (std::fabs(w) < 1.0e-12) {
				return false;
			}
			w = -(c[0].x * dir.x + c[0].y * dir.y) / w;
			w = glm::clamp(w, 0.0, 1.0);

			// compute v on the curve segment.
			// double v = v0 * (1 - w) + v1 * w;
			double v = glm::mix(v0, v1, w);

			double radius_value = radius * radius_f(v);

			// compare x-y distances.
			auto p = original.evaluate(v);
			double distanceSquared = p.x * p.x + p.y * p.y;
			if (distanceSquared >= radius_value * radius_value || p.z <= glm::epsilon<double>()) {
				return false;
			}

			// compare z distances.
			if (*tmin < p.z) {
				return false;
			}

			// tminは、コリジョンが成立するセグメント内での"中心"の最小を指す。ベジエの表面ではない
			*tmin = p.z;

			// intersection->h = glm::sqrt(distanceSquared) / radius;
			intersection->h = distanceRayRay(p, original.tangent(v)) / radius_value;
			intersection->bezier_t = v;

			Vec2 to_zero(-p.x, -p.y);
			Vec2 rhs_dir(dir.y, -dir.x);
			/*
			どちらを横切るか？
			lhs
			-(ray)-------->
			rhs
			*/
			bool isRhs = 0.0 > glm::dot(to_zero, rhs_dir);
			if (isRhs) {
				intersection->h = -intersection->h;
			}

			return true;
		}
		else {
			//ofSetColor(255);
			//drawAABB(b);

			double vm = (v0 + v1) * 0.5;
			BezierQuadratic3D cl, cr;
			std::tie(cl, cr) = c.split(0.5);

			bool intersect_L = intersect_bezier(depth - 1, radius, radiusSq, original, cl, v0, vm, tmin, intersection);
			bool intersect_R = intersect_bezier(depth - 1, radius, radiusSq, original, cr, vm, v1, tmin, intersection);
			return intersect_L || intersect_R;
		}
	}
	
	// 通常版
	inline bool intersect_bezier_standard(int depth, double radius, double radiusSq, const BezierQuadratic3D &original, const BezierQuadratic3D &c, double v0, double v1, double *tmin, CurveIntersection *intersection)
	{
		auto b = c.boundingBox();
		if (b.min_position.z >= *tmin || b.max_position.z <= glm::epsilon<double>()
			|| b.min_position.x >= radius || b.max_position.x <= -radius
			|| b.min_position.y >= radius || b.max_position.y <= -radius) {
			/// the bounding box does not overlap the square
			/// centered at O.
			//ofSetColor(64);
			//drawAABB(b);
			return false;
		} else if (depth == 0) {
			/// the maximum recursion depth is reached.
			auto dir = c[2] - c[0];

			// compute w on the line segment.
			double w = dir.x * dir.x + dir.y * dir.y;
			if (std::fabs(w) < 1.0e-12) {
				return false;
			}
			w = -(c[0].x * dir.x + c[0].y * dir.y) / w;
			w = glm::clamp(w, 0.0, 1.0);

			// compute v on the curve segment.
			// double v = v0 * (1 - w) + v1 * w;
			double v = glm::mix(v0, v1, w);

			// compare x-y distances.
			auto p = original.evaluate(v);
			double distanceSquared = p.x * p.x + p.y * p.y;
			if (distanceSquared >= radiusSq || p.z <= glm::epsilon<double>()) {
				return false;
			}

			// compare z distances.
			if (*tmin < p.z) {
				return false;
			}

			// tminは、コリジョンが成立するセグメント内での"中心"の最小を指す。ベジエの表面ではない
			*tmin = p.z;

			// intersection->h = glm::sqrt(distanceSquared) / radius;
			intersection->h = distanceRayRay(p, original.tangent(v)) / radius;
			intersection->bezier_t = v;

			Vec2 to_zero(-p.x, -p.y);
			Vec2 rhs_dir(dir.y, -dir.x);
			/*
			どちらを横切るか？
			  lhs
			-(ray)-------->
			  rhs
			*/
			bool isRhs = 0.0 > glm::dot(to_zero, rhs_dir);
			if (isRhs) {
				intersection->h = -intersection->h;
			}

			return true;
		}
		else {
			//ofSetColor(255);
			//drawAABB(b);

			double vm = (v0 + v1) * 0.5;
			BezierQuadratic3D cl, cr;
			std::tie(cl, cr) = c.split(0.5);

			bool intersect_L = intersect_bezier(depth - 1, radius, radiusSq, original, cl, v0, vm, tmin, intersection);
			bool intersect_R = intersect_bezier(depth - 1, radius, radiusSq, original, cr, vm, v1, tmin, intersection);
			return intersect_L || intersect_R;
		}
	}

	/*
	  カルダノの公式を使って求めるもの
	　残念ながら、遅い。
	*/
	inline bool intersect_bezier_cardano(int , double radius, double radiusSq, const BezierQuadratic3D &original, const BezierQuadratic3D &, double , double , double *tmin, CurveIntersection *intersection) {
		auto p0 = rt::Vec2(original[0]);
		auto p1 = rt::Vec2(original[1]);
		auto p2 = rt::Vec2(original[2]);
		auto A = p1 - p0;
		auto B = p2 - p1 - A;
		auto M = rt::Vec2(0.0, 0.0);

		auto M_tap = p0 - M;

		auto a = glm::dot(B, B);
		auto b = 3.0 * glm::dot(A, B);
		auto c = 2.0 * glm::dot(A, A) + glm::dot(M_tap, B);
		auto d = glm::dot(M_tap, A);

		double t[3];
		int N = cardano_solve(a, b, c, d, t);

		bool intersected = false;
		for (int i = 0; i < N; ++i) {
			double this_t = glm::clamp(t[i], 0.0, 1.0);
			auto this_P = original.evaluate(this_t);
			if (this_P.z < 0.0 || *tmin < this_P.z) {
				continue;
			}
			double distanceSq = glm::distance2(rt::Vec2(this_P), M);
			if (radiusSq < distanceSq) {
				continue;
			}

			*tmin = this_P.z;
			intersection->bezier_t = this_t;
			intersection->h = glm::sqrt(distanceSq) / radius;

			Vec2 dir = Vec2(original.tangent(this_t));
			Vec2 to_zero = -Vec2(this_P);
			Vec2 rhs_dir(dir.y, -dir.x);
			/*
			どちらを横切るか？
			lhs
			-(ray)-------->
			rhs
			*/
			bool isRhs = 0.0 > glm::dot(to_zero, rhs_dir);
			if (isRhs) {
				intersection->h = -intersection->h;
			}

			intersected = true;
		}
		return intersected;
	}
}