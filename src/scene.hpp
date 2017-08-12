#pragma once

#include <memory>
#include <chrono>
#include <functional>
#include <random>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <mapbox/variant.hpp>

#include "peseudo_random.hpp"
#include "geometry.hpp"
#include "bezier.hpp"
//#include "bvh.hpp"
//#include "bezier.hpp"
//#include "bezier_bvh.hpp"
//#include "halton_sampler.h"
#include "hairfur.hpp"

/*
約束

光源はlambertianのみ

intersectの規則
・tminを引数に入れているときは、それより近い場合にのみヒット
*/

namespace rt {
	class LambertianMaterial {
	public:
		LambertianMaterial() {}
		LambertianMaterial(Vec3 e, Vec3 r) : Le(e), R(r) {}
		Vec3 Le;
		Vec3 R;

		bool isEmissive() const {
			return glm::any(glm::greaterThanEqual(Le, Vec3(glm::epsilon<double>())));
		}
	};

	class SpecularMaterial {
	public:
		
	};

	class Fur {
	public:
		Vec3 tangent;
		double h = 0.0;
	};

	using Material = mapbox::util::variant<LambertianMaterial, SpecularMaterial, Fur>;

	inline Vec3 uniform_on_triangle(rt::PeseudoRandom *random, const Vec3 &a, const Vec3 &b, const Vec3 &c) {
		auto u = random->uniform();
		auto v = random->uniform();
		auto min_value = std::min(u, v);
		auto max_value = std::max(u, v);
		return a * min_value + b * (1.0 - max_value) + c * (max_value - min_value);
	}
	
	inline Vec3 refract(const Vec3 &I, const Vec3 &N, double eta, bool *reflection) {
		double NoI = glm::dot(N, I);
		double k = 1.0 - eta * eta * (1.0 - NoI * NoI);
		if (k <= 0.0) {
			*reflection = true;
			return I - 2.0 * N * NoI;
		}
		*reflection = false;
		return eta * I - (eta * NoI + glm::sqrt(k)) * N;
	}
	// フレネル (Schlick近似)
	// mix(refract, reflect, value);
	inline double fresnel_schlick(double NoL, double f0) {
		return f0 + (1.0 - f0) * glm::pow(1.0 - NoL, 5.0);
	}

	// directionを yaxisに基底変換
	inline Vec3 basis_transform(const Vec3 &direction, const Vec3 &yaxis) {
		Vec3 xaxis;
		Vec3 zaxis;
		if (0.999 < glm::abs(yaxis.z)) {
			xaxis = glm::normalize(glm::cross(Vec3(0.0, -1.0, 0.0), yaxis));
		}
		else {
			xaxis = glm::normalize(glm::cross(Vec3(0.0, 0.0, 1.0), yaxis));
		}
		zaxis = glm::cross(xaxis, yaxis);
		return direction.x * xaxis + direction.y * yaxis + direction.z * zaxis;
	}

	// zが上の座標系に移動する行列
	inline Mat3 to_bxdf_basis_transform(const Vec3 &n) {
		Vec3 xaxis;
		Vec3 zaxis = n;
		Vec3 yaxis;
		if (0.999 < glm::abs(zaxis.z)) {
			xaxis = glm::normalize(glm::cross(Vec3(0.0, 1.0, 0.0), zaxis));
		}
		else {
			xaxis = glm::normalize(glm::cross(Vec3(0.0, 0.0, 1.0), zaxis));
		}
		yaxis = glm::cross(zaxis, xaxis);
		return glm::transpose(Mat3(xaxis, yaxis, zaxis));
	}
	inline Mat3 from_bxdf_basis_transform(const Vec3 &n) {
		return glm::transpose(to_bxdf_basis_transform(n));
	}

	inline Vec3 from_bxdf(const Vec3 &n, const Vec3 &bxdf_dir) {
		return glm::transpose(to_bxdf_basis_transform(n)) * bxdf_dir;
	}

	inline Vec3 uniform_on_sphere(const Sphere &s, Vec3 *n, PeseudoRandom *random) {
		Vec3 d;
		double sq = 0.0;
		do {
			d.x = random->uniform(-1.0, 1.0);
			d.y = random->uniform(-1.0, 1.0);
			d.z = random->uniform(-1.0, 1.0);
			sq = glm::length2(d);
		} while (sq < 0.0001 || 1.0 < sq);
		d /= glm::sqrt(sq);
		*n = d;
		return s.center + s.radius * d;
	}
	inline Vec3 uniform_on_unit_sphere(PeseudoRandom *random) {
		Vec3 d;
		double sq = 0.0;
		do {
			d.x = random->uniform(-1.0, 1.0);
			d.y = random->uniform(-1.0, 1.0);
			d.z = random->uniform(-1.0, 1.0);

			sq = glm::length2(d);
		} while (sq < 0.0001 || 1.0 < sq);
		d /= glm::sqrt(sq);
		return d;
	}

	inline Vec2 uniform_in_unit_circle(PeseudoRandom *random) {
		Vec2 d;
		double sq = 0.0;
		do {
			d.x = random->uniform(-1.0, 1.0);
			d.y = random->uniform(-1.0, 1.0);
			sq = glm::length2(d);
		} while (1.0 < sq);
		return d;
	}

	// yが上になっちゃっているバージョン
	inline Vec3 sample_cosine_weighted_hemisphere(PeseudoRandom *random, double *pdf) {
		double u1 = random->uniform();
		double u2 = random->uniform();
		double r = glm::sqrt(u1);
		double phi = glm::two_pi<double>() * u2;
		Vec3 sample(r * glm::cos(phi), glm::sqrt(1.0 - u1), r * glm::sin(phi));
		*pdf = glm::dot(sample, Vec3(0.0, 1.0, 0.0)) * glm::one_over_pi<double>();
		return sample;
	}

	// 一般的極座標系でzが法線
	inline double cosine_weighted_hemisphere_pdf_brdf(Vec3 dir) {
		return dir.z * glm::one_over_pi<double>();
	}
	// 一般的極座標系でzが法線
	inline Vec3 sample_cosine_weighted_hemisphere_brdf(PeseudoRandom *random) {
		double u1 = random->uniform();
		double u2 = random->uniform();
		double r = glm::sqrt(u1);
		double phi = glm::two_pi<double>() * u2;
		Vec3 sample(r * glm::cos(phi), r * glm::sin(phi), glm::sqrt(1.0 - u1));
		return sample;
	}
	inline double abs_cos_theta_bxdf(Vec3 dir) {
		return glm::abs(dir.z);
	}

	// 面積が均一になるようなサンプリング
	class AreaUniformSampler {
	public:
		AreaUniformSampler(const std::vector<double> &areas) {
			double area = 0.0;
			for (int i = 0; i < areas.size(); ++i) {
				area += areas[i];
				_cumulativeAreas.push_back(area);
			}
			_area = area;
			_areas = areas;
		}
		int sample(PeseudoRandom *random) const {
			double area_at = random->uniform(0.0, _area);
			auto it = std::upper_bound(_cumulativeAreas.begin(), _cumulativeAreas.end(), area_at);
			std::size_t index = std::distance(_cumulativeAreas.begin(), it);
			index = std::min(index, _cumulativeAreas.size() - 1);
			return (int)index;
		}
		double area() const {
			return _area;
		}
		double prob(int index) const {
			return _areas[index] / _area;
		}
		double _area = 0.0;
		std::vector<double> _areas;
		std::vector<double> _cumulativeAreas;
	};


	struct CameraSetting {
		double _fov = glm::radians(45.0);

		Vec3 _eye = Vec3(0.0, 0.0, 1.0);
		Vec3 _lookat = Vec3(0.0, 0.0, 0.0);
		Vec3 _up = Vec3(0.0, 1.0, 0.0);

		int _imageWidth = 1024;
		int _imageHeight = 768;

		double _lensRadius = 0.1;
		double _focalLength = 3.0;
	};

	class PinholeCamera {
	public:
		PinholeCamera(CameraSetting setting) {
			_setting = setting;

			double w = _setting._imageWidth;
			double h = _setting._imageHeight;
			_proj = glm::perspectiveFov(_setting._fov, w, h, 1.0, 100.0);

			auto vp = Vec4(0.0, 0.0, w - 1, h - 1);
			_origin = _setting._eye;

			_view = glm::lookAt(_setting._eye, _setting._lookat, _setting._up);

			_UL = glm::unProject(Vec3(0.0, h - 1, 0.0), _view, _proj, vp);
			auto r = glm::unProject(Vec3(w - 1, h - 1, 0.0), _view, _proj, vp);
			auto b = glm::unProject(Vec3(0.0, 0.0, 0.0), _view, _proj, vp);


			_dirRight = r - _UL;
			_dirBottom = b - _UL;
			_dirFront = glm::normalize(glm::cross(_dirRight, _dirBottom));
		}

		Ray generateRay(double filmx, double filmy) const {
			Ray ray;
			ray.d = glm::normalize(_UL + _dirRight * (filmx / (_setting._imageWidth - 1)) + _dirBottom * (filmy / (_setting._imageHeight - 1)) - _origin);
			ray.o = _origin;
			return ray;
		}

		// レンズの後ろの座標
		Vec3 filmToWorld(double filmx, double filmy) const {
			auto ul = _UL + 2.0 * (_origin - _UL);
			return ul - _dirRight * (filmx / (_setting._imageWidth - 1)) - _dirBottom * (filmy / (_setting._imageHeight - 1));
		}

		int imageWidth() const {
			return _setting._imageWidth;
		}
		int imageHeight() const {
			return _setting._imageHeight;
		}

		Vec3 front() const {
			return _dirFront;
		}
		Vec3 origin() const {
			return _origin;
		}

		// {filmx, fimly}
		// ピクセル座標へ
		Vec2 projectToFilm(Vec3 p) const {
			auto vp = Vec4(0.0, 0.0, _setting._imageWidth - 1, _setting._imageHeight - 1);
			auto projected = Vec2(glm::project(p, _view, _proj, vp));
			projected.y = _setting._imageHeight - projected.y - 1;
			return projected;
		}
		double filmArea() const {
			return glm::length(_dirRight) * glm::length(_dirBottom);
		}
		double lensArea() const {
			return glm::pi<double>() * _setting._lensRadius * _setting._lensRadius;
		}

		// (u, v)が対象のピクセルインデックス
		// h(x, y)を返す
		double filter(double x, double y, int u, int v) const {
			double value = _setting._imageWidth * _setting._imageHeight / filmArea();
			return glm::abs(x - u) < 0.5 && glm::abs(y - v) < 0.5 ? value : 0.0;
		}

		Vec3 sampleLens(PeseudoRandom *random) const {
			double r = _setting._lensRadius;
			Vec3 xaxis = r * glm::normalize(_dirRight);
			Vec3 yaxis = r * glm::normalize(-_dirBottom);
			auto xy = uniform_in_unit_circle(random);
			return origin() + xaxis * xy.x + yaxis * xy.y;
		}

		CameraSetting _setting;

		Mat4 _proj;
		Mat4 _view;

		Vec3 _origin;

		Vec3 _UL;
		Vec3 _dirRight;
		Vec3 _dirBottom;
		Vec3 _dirFront;  /* normalized */
	};

	class SceneElement {
	public:
		virtual ~SceneElement() {}

		virtual bool intersect(const Ray &ray, Material *mat, Intersection *intersection, double *tmin) const = 0;
		
		virtual double emissiveArea() const = 0;
		virtual void sampleEmissive(Vec3 *p, Vec3 *n, Vec3 *emissiveRadiance, PeseudoRandom *random) const = 0;
		virtual void drawPreview(std::function<void(const Vec3 &, const Vec3 &)> drawLine) const {}
	};

	// 
	class PolygonSceneElement : public SceneElement {
	public:
		PolygonSceneElement(const std::vector<Vec3> &vertices, std::vector<int> const &indices, Material material, bool backEmissive)
		:_vertices(vertices)
		,_indices(indices)
		,_material(material)
		,_backEmissive(backEmissive) {
			std::vector<double> areas;
			double area = 0.0;
			for (int i = 0; i < _indices.size(); i += 3) {
				const Vec3 &v0 = _vertices[_indices[i + 0]];
				const Vec3 &v1 = _vertices[_indices[i + 1]];
				const Vec3 &v2 = _vertices[_indices[i + 2]];
				areas.push_back(triangle_area(v0, v1, v2));
			}
			_areaUniformSampler = std::unique_ptr<AreaUniformSampler>(new AreaUniformSampler(areas));
		}

		bool intersect(const Ray &ray, Material *mat, Intersection *intersection, double *tmin) const override {
			bool intersected = false;
			for (int i = 0; i < _indices.size(); i += 3) {
				const Vec3 &v0 = _vertices[_indices[i + 0]];
				const Vec3 &v1 = _vertices[_indices[i + 1]];
				const Vec3 &v2 = _vertices[_indices[i + 2]];
				TriangleIntersection thisIntersection;
				if (intersect_triangle(ray, v0, v1, v2, &thisIntersection, tmin)) {
					*mat = _material;

					if (_backEmissive == false && thisIntersection.isBack) {
						if (_material.is<LambertianMaterial>()) {
							auto m = _material.get<LambertianMaterial>();
							m.Le = Vec3(0.0);
							*mat = m;
						}
					}

					*intersection = thisIntersection;
					intersected = true;
				}
			}
			return intersected;
		}

		double emissiveArea() const override {
			if (_material.is<LambertianMaterial>()) {
				auto mat = _material.get<LambertianMaterial>();
				if (mat.isEmissive()) {
					if (_backEmissive) {
						return _areaUniformSampler->area() * 2.0;
					}
					return _areaUniformSampler->area();
				}
			}
			return 0.0;
		}

		void sampleEmissive(Vec3 *p, Vec3 *n, Vec3 *emissiveRadiance, PeseudoRandom *random) const override {
			int index = _areaUniformSampler->sample(random);
			
			auto p0 = _vertices[_indices[index * 3]];
			auto p1 = _vertices[_indices[index * 3 + 1]];
			auto p2 = _vertices[_indices[index * 3 + 2]];
			*p = uniform_on_triangle(random,
				p0,
				p1,
				p2
			);
			if (_backEmissive) {
				*n = triangle_normal(p0, p1, p2, random->uniform() < 0.5);
			}
			else {
				*n = triangle_normal(p0, p1, p2, false);
			}

			if (_material.is<LambertianMaterial>()) {
				*emissiveRadiance = _material.get<LambertianMaterial>().Le;
			}
			else {
				assert(0);
			}
		}
		void drawPreview(std::function<void(const Vec3 &, const Vec3 &)> drawLine) const override {
			for (int i = 0; i < _indices.size(); i += 3) {
				const Vec3 &v0 = _vertices[_indices[i + 0]];
				const Vec3 &v1 = _vertices[_indices[i + 1]];
				const Vec3 &v2 = _vertices[_indices[i + 2]];
				drawLine(v0, v1);
				drawLine(v1, v2);
				drawLine(v2, v0);
			}
		}

		std::vector<Vec3> _vertices;
		std::vector<int> _indices;

		Material _material;

		// 両面の対応はすべてSceneElementの中で対応可能
		bool _backEmissive = false;
		
		// ライトサンプル用
		std::unique_ptr<AreaUniformSampler> _areaUniformSampler;
	};

	class BezierSceneElement : public SceneElement {
	public:
		BezierSceneElement(std::vector<BezierQuadratic3D> beziers)
			:_beziers(beziers) {
		}

		bool intersect(const Ray &ray, Material *mat, Intersection *intersection, double *tmin) const override {
			double radius = 0.02;
			auto projection = rt::ray_projection(ray.o, ray.d);

			bool intersected = false;
			for (int i = 0; i < _beziers.size(); ++i) {
				auto bezier = _beziers[i].transform(projection);

				rt::CurveIntersection thisIntersection;
				bool intersected = rt::intersect_bezier(7, radius, radius * radius, bezier, bezier, 0.0, 1.0, tmin, &thisIntersection);
				Vec3 tangent;

				// origin rejection
				if (intersected) {
					tangent = bezier.tangent(thisIntersection.bezier_t);
					auto p = bezier.evaluate(thisIntersection.bezier_t);
					if (rt::distanceSqPointRay(rt::Vec3(0.0), p, tangent) < radius * radius) {
						intersected = false;
					}
				}
				if (intersected) {
					Fur fur;
					fur.h = thisIntersection.h;
					fur.tangent = glm::normalize(tangent);
					*mat = fur;
					*intersection = Intersection();
					intersected = true;
				}
			}
			return intersected;
		}

		double emissiveArea() const override {
			return 0.0;
		}

		void sampleEmissive(Vec3 *p, Vec3 *n, Vec3 *emissiveRadiance, PeseudoRandom *random) const override {
			assert(0);
		}
		void drawPreview(std::function<void(const Vec3 &, const Vec3 &)> drawLine) const override {
			//for (int i = 0; i < _indices.size(); i += 3) {
			//	const Vec3 &v0 = _vertices[_indices[i + 0]];
			//	const Vec3 &v1 = _vertices[_indices[i + 1]];
			//	const Vec3 &v2 = _vertices[_indices[i + 2]];
			//	drawLine(v0, v1);
			//	drawLine(v1, v2);
			//	drawLine(v2, v0);
			//}
		}

		std::vector<BezierQuadratic3D> _beziers;
	};

	//class BVHPolygonSceneElement : public SceneElement {
	//public:
	//	BVHPolygonSceneElement(const std::vector<Vec3> &vertices, std::vector<int> const &indices, Material material, bool backEmissive)
	//		: _vertices(vertices)
	//		, _indices(indices)
	//		, _material(material)
	//		, _backEmissive(backEmissive) {
	//		std::vector<double> areas;
	//		double area = 0.0;
	//		for (int i = 0; i < _indices.size(); i += 3) {
	//			const Vec3 &v0 = _vertices[_indices[i + 0]];
	//			const Vec3 &v1 = _vertices[_indices[i + 1]];
	//			const Vec3 &v2 = _vertices[_indices[i + 2]];
	//			areas.push_back(triangle_area(v0, v1, v2));
	//		}
	//		_areaUniformSampler = std::unique_ptr<AreaUniformSampler>(new AreaUniformSampler(areas));
	//		_bvh = std::unique_ptr<BVH>(new BVH(_vertices, _indices));
	//	}

	//	bool intersect(const Ray &ray, Material *mat, Intersection *intersection, TracingContext &context, double *tmin) const override {
	//		BVHIntersection thisIntersection;
	//		if (_bvh->intersect(ray, &thisIntersection, tmin)) {
	//			*mat = _material;

	//			if (_backEmissive == false && thisIntersection.isBack) {
	//				if (_material.is<LambertianMaterial>()) {
	//					auto m = _material.get<LambertianMaterial>();
	//					m.Le = Vec3(0.0);
	//					*mat = m;
	//				}
	//			}

	//			*intersection = thisIntersection;
	//			return true;
	//		}
	//		return false;
	//	}

	//	double emissiveArea() const override {
	//		if (_material.is<LambertianMaterial>()) {
	//			auto mat = _material.get<LambertianMaterial>();
	//			if (mat.isEmissive()) {
	//				if (_backEmissive) {
	//					return _areaUniformSampler->area() * 2.0;
	//				}
	//				return _areaUniformSampler->area();
	//			}
	//		}
	//		return 0.0;
	//	}

	//	void sampleEmissive(Vec3 *p, Vec3 *n, Vec3 *emissiveRadiance, PeseudoRandom *random) const override {
	//		int index = _areaUniformSampler->sample(random);

	//		auto p0 = _vertices[_indices[index * 3]];
	//		auto p1 = _vertices[_indices[index * 3 + 1]];
	//		auto p2 = _vertices[_indices[index * 3 + 2]];
	//		*p = uniform_on_triangle(random,
	//			p0,
	//			p1,
	//			p2
	//		);
	//		if (_backEmissive) {
	//			*n = triangle_normal(p0, p1, p2, random->uniform() < 0.5);
	//		}
	//		else {
	//			*n = triangle_normal(p0, p1, p2, false);
	//		}

	//		if (_material.is<LambertianMaterial>()) {
	//			*emissiveRadiance = _material.get<LambertianMaterial>().Le;
	//		}
	//		else {
	//			assert(0);
	//		}
	//	}
	//	void drawPreview(std::function<void(const Vec3 &, const Vec3 &)> drawLine) const override {
	//		for (int i = 0; i < _indices.size(); i += 3) {
	//			const Vec3 &v0 = _vertices[_indices[i + 0]];
	//			const Vec3 &v1 = _vertices[_indices[i + 1]];
	//			const Vec3 &v2 = _vertices[_indices[i + 2]];
	//			drawLine(v0, v1);
	//			drawLine(v1, v2);
	//			drawLine(v2, v0);
	//		}
	//	}
	//	std::unique_ptr<BVH> _bvh;
	//	std::vector<Vec3> _vertices;
	//	std::vector<int> _indices;

	//	Material _material;

	//	// 両面の対応はすべてSceneElementの中で対応可能
	//	bool _backEmissive = false;

	//	// ライトサンプル用
	//	std::unique_ptr<AreaUniformSampler> _areaUniformSampler;
	//};


	//class BezierSceneElement : public SceneElement {
	//public:
	//	inline Vec3 uniform_on_unit_sphere(Halton_sampler *sampler, int *index) {
	//		Vec3 d;
	//		double sq = 0.0;
	//		do {
	//			d.x = glm::mix(-1.0, 1.0, sampler->sample(1, *index));
	//			d.y = glm::mix(-1.0, 1.0, sampler->sample(2, *index));
	//			d.z = glm::mix(-1.0, 1.0, sampler->sample(3, *index));

	//			(*index)++;

	//			sq = glm::length2(d);
	//		} while (sq < 0.0001 || 1.0 < sq);
	//		d /= glm::sqrt(sq);
	//		return d;
	//	}

	//	//inline std::vector<BezierCurve3D> basic_mofumofu() {
	//	//	static float fur_length = 0.15f;
	//	//	static float fur_softness = 1.0;

	//	//	Halton_sampler sampler;
	//	//	std::mt19937 engine = std::mt19937(3);
	//	//	sampler.init_random(engine);

	//	//	double scale = fur_length;
	//	//	double softness = fur_softness;

	//	//	std::vector<BezierCurve3D> r;
	//	//	int sampler_index = 0;
	//	//	rt::Xor random;
	//	//	for (int i = 0; i < 4000; ++i) {
	//	//		Vec3 n;
	//	//		n = uniform_on_unit_sphere(&sampler, &sampler_index);
	//	//		auto p = n * 0.4;

	//	//		auto r_axis = glm::normalize(glm::cross(Vec3(0.0, 1.0, 0.0), n));
	//	//		auto d = glm::pow(1.0 - glm::abs(-n.y), 0.7);
	//	//		auto rotation = d * softness;
	//	//		auto rn = glm::rotate(n, d * softness, r_axis) * 0.5;
	//	//		double sss = random.uniform(0.7, 1.4);
	//	//		Vec3 ofs(random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05));
	//	//		BezierCurve3D bz(
	//	//			p,
	//	//			p + (n * 0.1) * scale * sss,
	//	//			p + (n - rn * 0.2) * scale * sss,
	//	//			p + (n + rn) * scale * sss + ofs
	//	//		);
	//	//		r.push_back(bz);
	//	//	}
	//	//	return r;
	//	//}
	//	inline std::vector<BezierQuadratic3D> basic_mofumofu() {
	//		static float fur_length = 0.15f;
	//		static float fur_softness = 1.0;

	//		Halton_sampler sampler;
	//		std::mt19937 engine = std::mt19937(3);
	//		sampler.init_random(engine);

	//		double scale = fur_length;
	//		double softness = fur_softness;

	//		std::vector<BezierQuadratic3D> r;
	//		int sampler_index = 0;
	//		rt::Xor random;
	//		for (int i = 0; i < 4000; ++i) {
	//			Vec3 n;
	//			n = uniform_on_unit_sphere(&sampler, &sampler_index);
	//			auto p = n * 0.4;

	//			auto r_axis = glm::normalize(glm::cross(Vec3(0.0, 1.0, 0.0), n));
	//			auto d = glm::pow(1.0 - glm::abs(-n.y), 0.7);
	//			auto rotation = d * softness;
	//			auto rn = glm::rotate(n, d * softness, r_axis) * 0.5;
	//			double sss = random.uniform(0.7, 1.4);
	//			Vec3 ofs(random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05));
	//			BezierQuadratic3D bz(
	//				p,
	//				p + (n - rn * 0.2) * scale * sss,
	//				p + (n + rn) * scale * sss + ofs
	//			);
	//			r.push_back(bz);
	//		}
	//		return r;
	//	}

	//	BezierSceneElement() {
	//		auto curves = basic_mofumofu();
	//		std::vector<int> ids;
	//		for (int i = 0; i < curves.size(); ++i) {
	//			ids.push_back(i);
	//		}
	//		_bezier = std::shared_ptr<BezierBVH>(new BezierBVH(curves, ids, _radius));
	//	}

	//	bool intersect(const Ray &ray, Material *mat, Intersection *intersection, TracingContext &context, double *tmin) const override {
	//		// TODO 本当はだめ
	//		int skipindex = context.is<TracingContextBezier>() ? context.get<TracingContextBezier>().index : -1;
	//		//if (skipindex != -1) {
	//		//	return false;
	//		//}

	//		bool intersected = false;
	//		BzBVHIntersection thisIntersection;
	//		if (_bezier->intersect(ray, &thisIntersection, skipindex, tmin)) {
	//			intersected = true;

	//			TracingContextBezier ctx;
	//			ctx.index = thisIntersection.identifer;
	//			context = ctx;

	//			// * thisIntersection.closest_t
	//			HairFurMaterial m;
	//			m.h = glm::sqrt(thisIntersection.hSq) / (_radius );

	//			if (thisIntersection.isRhs) {
	//				m.h = -m.h;
	//			}

	//			m.tangent = _bezier->_curves[thisIntersection.identifer].tangent(thisIntersection.closest_t);
	//			m.tangent = glm::normalize(m.tangent);
	//			*mat = m;
	//			// *mat = LambertianMaterial(rt::Vec3(), rt::Vec3(0.8, 0.1, 0.1));

	//			Intersection thisIntersection;
	//			thisIntersection.isBack = false;
	//			thisIntersection.normal = -ray.d;
	//			*intersection = thisIntersection;
	//		}
	//		else {
	//			context = TracingContextBezier();
	//		}
	//		return intersected;
	//	}

	//	double emissiveArea() const override {
	//		return 0.0;
	//	}

	//	void sampleEmissive(Vec3 *p, Vec3 *n, Vec3 *emissiveRadiance, PeseudoRandom *random) const override {
	//		assert(0);
	//	}
	//	void drawPreview(std::function<void(const Vec3 &, const Vec3 &)> drawLine) const override {
	//		//for (int i = 0; i < _indices.size(); i += 3) {
	//		//	const Vec3 &v0 = _vertices[_indices[i + 0]];
	//		//	const Vec3 &v1 = _vertices[_indices[i + 1]];
	//		//	const Vec3 &v2 = _vertices[_indices[i + 2]];
	//		//	drawLine(v0, v1);
	//		//	drawLine(v1, v2);
	//		//	drawLine(v2, v0);
	//		//}
	//	}
	//	double _radius = 0.004;
	//	std::shared_ptr<BezierBVH> _bezier;
	//};

	// ここでは球は内側に発光しない
	class SphereSceneElement : public SceneElement {
	public:
		SphereSceneElement() {}
		SphereSceneElement(const Sphere &sphere, Material material):_sphere(sphere), _material(material) {}
		Sphere _sphere;
		Material _material;

		bool intersect(const Ray &ray, Material *mat, Intersection *intersection, double *tmin) const override {
			if (intersect_shere(ray, _sphere, intersection, tmin)) {
				*mat = _material;

				if (intersection->isBack) {
					if (_material.is<LambertianMaterial>()) {
						auto m = _material.get<LambertianMaterial>();
						m.Le = Vec3(0.0);
						*mat = m;
					}
				}
				return true;
			}
			return false;
		}

		double emissiveArea() const override {
			if (_material.is<LambertianMaterial>()) {
				auto mat = _material.get<LambertianMaterial>();
				if (mat.isEmissive()) {
					return 4.0 * glm::pi<double>() * _sphere.radius * _sphere.radius;
				}
			}
			return 0.0;
		}

		void sampleEmissive(Vec3 *p, Vec3 *n, Vec3 *emissiveRadiance, PeseudoRandom *random) const override {
			*p = uniform_on_sphere(_sphere, n, random);
			assert(_material.is<LambertianMaterial>());
			*emissiveRadiance = _material.get<LambertianMaterial>().Le;
		}
		void drawPreview(std::function<void(const Vec3 &, const Vec3 &)> drawLine) const override {
			int N = 20;
			double step_theta = glm::radians(360.0) / N;
			for (int i = 0; i < N; ++i) {
				Vec3 d0(glm::cos(step_theta * i), 0.0, glm::sin(step_theta * i));
				Vec3 d1(glm::cos(step_theta * (i + 1)), 0.0, glm::sin(step_theta * (i + 1)));
				drawLine(_sphere.center + d0 * _sphere.radius, _sphere.center + d1 * _sphere.radius);
			}
			for (int i = 0; i < N; ++i) {
				Vec3 d0(glm::cos(step_theta * i), glm::sin(step_theta * i), 0.0);
				Vec3 d1(glm::cos(step_theta * (i + 1)), glm::sin(step_theta * (i + 1)), 0.0);
				drawLine(_sphere.center + d0 * _sphere.radius, _sphere.center + d1 * _sphere.radius);
			}
			for (int i = 0; i < N; ++i) {
				Vec3 d0(0.0, glm::cos(step_theta * i), glm::sin(step_theta * i));
				Vec3 d1(0.0, glm::cos(step_theta * (i + 1)), glm::sin(step_theta * (i + 1)));
				drawLine(_sphere.center + d0 * _sphere.radius, _sphere.center + d1 * _sphere.radius);
			}
		}
	};

	class Scene {
	public:
		Scene(const CameraSetting &cameraSetting, const std::vector<std::shared_ptr<SceneElement>> &sceneElements)
		:_cameraSetting(cameraSetting)
		,_sceneElements(sceneElements)
		,_camera(new PinholeCamera(cameraSetting)) {
			for (int i = 0; i < _sceneElements.size(); ++i) {
				if (glm::epsilon<double>() <= _sceneElements[i]->emissiveArea()) {
					_emissiveElements.push_back(_sceneElements[i]);
				}
			}

			std::vector<double> areas;
			double area = 0.0;
			for (int i = 0; i < _emissiveElements.size(); ++i) {
				areas.push_back(_emissiveElements[i]->emissiveArea());
			}
			_areaUniformSampler = std::unique_ptr<AreaUniformSampler>(new AreaUniformSampler(areas));
		}

		std::shared_ptr<Scene> addElement(std::shared_ptr<SceneElement> element) const {
			auto cp = _sceneElements;
			cp.push_back(element);
			return std::shared_ptr<Scene>(new Scene(_cameraSetting, cp));
		}

		bool intersect(const Ray &ray, Material *mat, Intersection *intersection, double *tmin) const {
			bool intersected = false;
			for (int i = 0; i < _sceneElements.size(); ++i) {
				if (_sceneElements[i]->intersect(ray, mat, intersection, tmin)) {
					intersected = true;
				}
			}
			return intersected;
		}

		// エリアに均等であり、areaは常にライト全体の面積を返す
		void sampleEmissive(Vec3 *p, Vec3 *n, Vec3 *emissiveRadiance, double *area, PeseudoRandom *random) const {
			*area = _areaUniformSampler->area();
			int index = _areaUniformSampler->sample(random);
			_emissiveElements[index]->sampleEmissive(p, n, emissiveRadiance, random);
		}
		double emissiveArea() const {
			return _areaUniformSampler->area();
		}

		void drawPreview(std::function<void(const Vec3 &, const Vec3 &)> drawLine) const {
			{
				auto ray = _camera->generateRay(0, 0);
				drawLine(ray.o, ray.o + ray.d * 100.0);
			}
			{
				auto ray = _camera->generateRay(0, _cameraSetting._imageHeight);
				drawLine(ray.o, ray.o + ray.d * 100.0);
			}
			{
				auto ray = _camera->generateRay(_cameraSetting._imageWidth - 1, 0);
				drawLine(ray.o, ray.o + ray.d * 100.0);
			}
			{
				auto ray = _camera->generateRay(_cameraSetting._imageWidth, _cameraSetting._imageHeight - 1);
				drawLine(ray.o, ray.o + ray.d * 100.0);
			}

			{
				drawLine(_camera->_UL, _camera->_UL + _camera->_dirRight);
				drawLine(_camera->_UL, _camera->_UL + _camera->_dirBottom);
			}

			{
				double r = _camera->_setting._lensRadius;
				Vec3 xaxis = r * glm::normalize(_camera->_dirRight);
				Vec3 yaxis = r * glm::normalize(-_camera->_dirBottom);
				double step_theta = glm::two_pi<double>() / 20.0;
				for (int i = 0; i < 20; ++i) {
					
					auto p1 = _camera->origin() + xaxis * glm::cos(step_theta * i) + yaxis * glm::sin(step_theta * i);
					auto p2 = _camera->origin() + xaxis * glm::cos(step_theta * (i + 1)) + yaxis * glm::sin(step_theta * (i + 1));
					drawLine(p1, p2);
				}
			}

			for (int i = 0; i < _sceneElements.size(); ++i) {
				_sceneElements[i]->drawPreview(drawLine);
			}
		}
		std::shared_ptr<const PinholeCamera> camera() const {
			return _camera;
		}

		CameraSetting _cameraSetting;
		std::shared_ptr<PinholeCamera> _camera;
		std::vector<std::shared_ptr<SceneElement>> _sceneElements;

		// ライトサンプル用
		std::vector<std::shared_ptr<SceneElement>> _emissiveElements;
		std::unique_ptr<AreaUniformSampler> _areaUniformSampler;
	};



}
