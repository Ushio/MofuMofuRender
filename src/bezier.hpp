#pragma once

#include <array>
#include "geometry.hpp"
#include "aabb.hpp"
#include "catch.hpp"

namespace rt {
	/*
	レイとレイの最近傍距離を返す
	レイは正規化されている必要はない
	*/
	inline double distanceRayRay(rt::Vec3 p1, rt::Vec3 d1, rt::Vec3 p2, rt::Vec3 d2) {
		Vec3 r = p1 - p2;
		double a = glm::dot(d1, d1);
		double e = glm::dot(d2, d2);
		double f = glm::dot(d2, r);

		double b = glm::dot(d1, d2);
		double denom = a*e - b*b; 

		double c = glm::dot(d1, r);
		double s, t;
		if (glm::epsilon<double>() < denom) {
			s = (b*f - c*e) / denom;
		}
		else {
			s = 0.0f;
		}
		t = (b*s + f) / e;

		auto c1 = p1 + d1 * s;
		auto c2 = p2 + d2 * t;
		return glm::distance(c1, c2);
	}

	/*
	rt::Vec3 p1 = Vec3(0, 0, 0);
	rt::Vec3 d1 = Vec3(0, 0, 1);
	のケース
	*/
	inline double distanceRayRay(rt::Vec3 p2, rt::Vec3 d2) {
		Vec3 r = - p2;
		double e = glm::dot(d2, d2);
		double f = glm::dot(d2, r);

		double b = d2.z;
		double denom = e - b*b;

		double c = r.z;
		double s, t;
		if (glm::epsilon<double>() < denom) {
			s = (b*f - c*e) / denom;
		}
		else {
			s = 0.0f;
		}
		t = (b*s + f) / e;

		auto c1 = Vec3(0.0, 0.0, s);
		auto c2 = p2 + d2 * t;
		return glm::distance(c1, c2);
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

	//inline Mat3 basisMatrix(Vec3 yaxis) {
	//	Vec3 xaxis;
	//	Vec3 zaxis;
	//	if (0.999 < glm::abs(yaxis.z)) {
	//		xaxis = glm::normalize(glm::cross(Vec3(0.0, -1.0, 0.0), yaxis));
	//	}
	//	else {
	//		xaxis = glm::normalize(glm::cross(Vec3(0.0, 0.0, 1.0), yaxis));
	//	}
	//	zaxis = glm::cross(xaxis, yaxis);
	//	return Mat3(xaxis, yaxis, zaxis);
	//}

	//inline void drawAABB(rt::AABB aabb) {
	//	auto sR = aabb.size();
	//	auto cR = aabb.center();
	//	ofNoFill();
	//	ofDrawBox(cR.x, cR.y, cR.z, sR.x, sR.y, sR.z);
	//	ofFill();
	//}

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
		BezierQuadratic3D transform(Mat4 m) const {
			return BezierQuadratic3D(
				m * Vec4(_ps[0], 1.0),
				m * Vec4(_ps[1], 1.0),
				m * Vec4(_ps[2], 1.0)
			);
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

		//void draw(bool draw_cp = false) {
		//	if (draw_cp) {
		//		ofSetColor(ofColor::red);
		//		ofDrawSphere(_ps[0].x, _ps[0].y, _ps[0].z, 0.02f);
		//		ofDrawSphere(_ps[2].x, _ps[2].y, _ps[2].z, 0.02f);

		//		ofSetColor(ofColor::orange);
		//		ofDrawSphere(_ps[1].x, _ps[1].y, _ps[1].z, 0.01f);
		//		ofLine(_ps[1].x, _ps[1].y, _ps[1].z, _ps[0].x, _ps[0].y, _ps[0].z);
		//		ofLine(_ps[1].x, _ps[1].y, _ps[1].z, _ps[2].x, _ps[2].y, _ps[2].z);
		//	}
		//	ofSetColor(ofColor::orange);

		//	ofPolyline line;
		//	int N = 10;
		//	for (int i = 0; i < N; ++i) {
		//		float t = (float)i / (N - 1);
		//		auto p = evaluate(t);

		//		line.addVertex(p.x, p.y, p.z);
		//	}

		//	line.draw();
		//}
		std::array<Vec3, 3> _ps;
	};

	
	struct CurveIntersection {
		double h = 0.0;
		double bezier_t = 0.0;
	};

	inline bool intersect_bezier(int depth, double radius, double radiusSq, BezierQuadratic3D original, BezierQuadratic3D c, double v0, double v1, double *tmin, CurveIntersection *intersection)
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
	//inline bool converge(int depth, double radius, double radiusSq, BezierQuadratic3D cw, BezierQuadratic3D c, double v0, double vn, double tmin, CurveIntersection *intersection)
	//{
	//	auto b = c.boundingBox();
	//	if (b.min_position.z >= tmin || b.max_position.z <= glm::epsilon<double>()
	//		|| b.min_position.x >= radius || b.max_position.x <= -radius
	//		|| b.min_position.y >= radius || b.max_position.y <= -radius) {
	//		/// the bounding box does not overlap the square
	//		/// centered at O.
	//		//ofSetColor(64);
	//		//drawAABB(b);
	//		return false;
	//	}
	//	else if (depth == 0) {
	//		/// the maximum recursion depth is reached.
	//		auto dir = c[2] - c[0];

	//		// compute w on the line segment.
	//		double w = dir.x * dir.x + dir.y * dir.y;
	//		if (fabs(w) < 1.0e-12) {
	//			return false;
	//		}
	//		w = -(c[0].x * dir.x + c[0].y * dir.y) / w;
	//		w = glm::clamp(w, 0.0, 1.0);
	//		// compute v on the curve segment.
	//		// double v = v0 * (1.0 - w) + vn * w;
	//		double v = glm::mix(v0, vn, w);

	//		// compare x-y distances.
	//		auto p = cw.evaluate(v);
	//		double hSq = p.x * p.x + p.y * p.y;
	//		if (hSq >= radiusSq || p.z <= glm::epsilon<double>()) {
	//			return false;
	//		}

	//		// compare z distances.
	//		if (tmin < p.z) {
	//			return false;
	//		}
	//		// *tmin = p.z;
	//		intersection->hSq = hSq;
	//		intersection->closest_t = v;
	//		intersection->tmin = p.z;

	//		Vec2 to_zero(-p.x, -p.y);
	//		Vec2 rhs_dir(dir.y, -dir.x);

	//		intersection->isRhs = 0.0 < glm::dot(to_zero, rhs_dir);

	//		return true;
	//	}
	//	else {
	//		//ofSetColor(255);
	//		//drawAABB(b);

	//		/// split the curve into two curves and process
	//		/// them.
	//		depth--;
	//		double vm = (v0 + vn) * 0.5;
	//		BezierQuadratic3D cl, cr;
	//		std::tie(cl, cr) = c.split(0.5);

	//		CurveIntersection lhsIsct;
	//		CurveIntersection rhsIsct;
	//		bool intersect_L = converge(depth, radius, radiusSq, cw, cl, v0, vm, tmin, &lhsIsct);
	//		if (intersect_L) {
	//			tmin = lhsIsct.tmin;
	//		}
	//		bool intersect_R = converge(depth, radius, radiusSq, cw, cr, vm, vn, tmin, &rhsIsct);

	//		if (intersect_L && intersect_R) {
	//			if (lhsIsct.hSq < rhsIsct.hSq) {
	//				*intersection = lhsIsct;
	//			}
	//			else {
	//				*intersection = rhsIsct;
	//			}
	//		}
	//		else if (intersect_L) {
	//			*intersection = lhsIsct;
	//		}
	//		else if (intersect_R) {
	//			*intersection = rhsIsct;
	//		}

	//		return intersect_L || intersect_R;
	//	}
	//}
	inline bool intersect_ray_local(BezierQuadratic3D c, double radius, double *tmin, CurveIntersection *intersection)
	{
		int depth = 4;
		CurveIntersection thisIntersection;
		double tmin_value = *tmin;
		bool b = converge(depth, radius, radius * radius, c, c, 0.0, 1.0, tmin_value, &thisIntersection);
		//bool b = false;
		if (b && thisIntersection.tmin < tmin_value) {
			*tmin = thisIntersection.tmin;
			*intersection = thisIntersection;
		}
		else {
			b = false;
		}
		return b;
	}

	// S1(s)=P1+s*(Q1-P1)およびS2(t)=P2+t*(Q2-P2)の
	// 最近接点C1およびC2を計算、Sおよびtを返す。
	// 関数の結果はS1(s)とS2(t)の間の距離の平方
	inline void closest_ray_segment(Vec3 p1, Vec3 q1, const Ray &ray, double *s, double *t) {
		// Point p1, Point q1
		Vec3 d1 = q1 - p1; // 線分S1の方向ベクトル
		Vec3 d2 = ray.d; // 線分S2の方向ベクトル
		Vec3 r = p1 - ray.o;
		double a = glm::dot(d1, d1); // 線分S1の距離の平方、常に非負
		double e = glm::dot(d2, d2); // 線分S2の距離の平方、常に非負
		double f = glm::dot(d2, r);

		double c = glm::dot(d1, r);

		// ここから一般的な縮退の場合を開始
		double b = glm::dot(d1, d2);
		double denom = a * e - b * b; // 常に非負

		// 線分が平行でない場合、L1上のL2に対する最近接点を計算、そして
		// 線分S1に対してクランプ。そうでない場合は任意s(ここでは0)を選択
		if (denom != 0.0) {
			*s = glm::clamp((b*f - c*e) / denom, 0.0, 1.0);
		} else {
			*s = 0.5;
		}

		// L2上のS1(s)に対する最近接点を以下を用いて計算
		// t = Dot((P1+D1*s)-P2,D2) / Dot(D2,D2) = (b*s + f) / e
		*t = (b* (*s) + f) / e;

		// tが[0,1]の中にあれば終了。そうでなければtをクランプ、sをtの新しい値に対して以下を用いて再計算
		// s = Dot((P2+D2*t)-P1,D1) / Dot(D1,D1)= (t*b - c) / a
		// そしてsを[0, 1]に対してクランプ
		if (*t < 0.0) {
			*t = 0.0;
			*s = glm::clamp(-c / a, 0.0, 1.0);
		}

		//else if (*t > 1.0) {
		//	*s = glm::clamp((b - c) / a, 0.0, 1.0);
		//}
		//c1 = p1 + d1 * s;
		//c2 = p2 + d2 * t;
		//return Dot(c1 - c2, c1 - c2);
	}

	*/
}