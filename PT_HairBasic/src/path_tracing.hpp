#pragma once

#include <tbb/tbb.h>
#include "scene.hpp"
namespace rt {
	class Image {
	public:
		Image(int w, int h) :_w(w), _h(h), _pixels(h*w) {
			for (int i = 0; i < _pixels.size(); ++i) {
				_pixels[i].random = Xor(i + 1);
				for (int j = 0; j < 100; ++j) {
					_pixels[i].random.generate();
				}
			}
		}
		int width() const {
			return _w;
		}
		int height() const {
			return _h;
		}

		void add(int x, int y, Vec3 c) {
			int index = y * _w + x;
			_pixels[index].color += c;
			_pixels[index].sample++;
		}

		struct Pixel {
			int sample = 0;
			Vec3 color;
			Xor random;
		};
		const Pixel *pixel(int x, int y) const {
			return _pixels.data() + y * _w + x;
		}
		Pixel *pixel(int x, int y) {
			return _pixels.data() + y * _w + x;
		}
	private:
		int _w = 0;
		int _h = 0;
		std::vector<Pixel> _pixels;
	};

	inline Vec3 radiance(std::shared_ptr<rt::Scene> scene, Ray ray, PeseudoRandom *random) {
		Vec3 Lo;
		Vec3 T(1.0);
		for (int i = 0; i < 10; ++i) {
			Material mat;
			Intersection intersection;
			double tmin = std::numeric_limits<double>::max();

			if (scene->intersect(ray, &mat, &intersection, &tmin)) {
				if(mat.is<LambertianMaterial>()) {
					auto material = mat.get<LambertianMaterial>();
					
					Vec3 sample = sample_cosine_weighted_hemisphere_brdf(random);
					double pdf_omega = cosine_weighted_hemisphere_pdf_brdf(sample);
					auto wi = from_bxdf(intersection.normal, sample);
					
					auto brdf = material.R * Vec3(glm::one_over_pi<double>());
					auto cos_term = abs_cos_theta_bxdf(sample);

					Lo += material.Le * T;

					T *= brdf * cos_term / pdf_omega;
					ray = Ray(ray.o + ray.d * tmin + wi * 0.000001, wi);
				} else if(mat.is<SpecularMaterial>()) {
					Vec3 wi = glm::reflect(ray.d, intersection.normal);
					ray = Ray(ray.o + ray.d * tmin + wi * 0.000001, wi);
				}
				else if (mat.is<Fur>()) {
					auto material = mat.get<Fur>();

					Vec3 wo = -ray.d;
					Vec3 wi = uniform_on_unit_sphere(random);

					auto to_bsdf = to_fur_bsdf_basis_transform(material.tangent);

					// アイ方向
					Vec3 bsdf_wo = to_bsdf * wo;

					// ライト方向
					Vec3 bsdf_wi = to_bsdf * wi;

					double pdf_omega = 1.0 / (4.0 * glm::pi<double>());
					Vec3 bsdf_value = fur_bsdf(bsdf_wi, bsdf_wo, material.h);
					T *= (bsdf_value / pdf_omega) * AbsCosThetaForFur(bsdf_wi);
					ray = Ray(ray.o + ray.d * tmin + wi * 0.000001, wi);
				}
			}
			else {
				break;
			}
		}
		return Lo;
	}

	inline void step_image(std::shared_ptr<rt::Scene> scene, Image &image) {
		auto camera = scene->camera();
		tbb::parallel_for(tbb::blocked_range<int>(0, camera->imageHeight()), [&](const tbb::blocked_range<int> &range) {
			for (int y = range.begin(); y < range.end(); ++y) {
				for (int x = 0; x < camera->imageWidth(); ++x) {
					PeseudoRandom *random = &image.pixel(x, y)->random;
					double dx = random->uniform(-0.5, 0.5);
					double dy = random->uniform(-0.5, 0.5);
					Ray ray = camera->generateRay(x + dx, y + dy);
					image.add(x, y, radiance(scene, ray, random));
				}
			}
		});
	}

	class PathTracer {
	public:
		PathTracer(std::shared_ptr<rt::Scene> scene)
			:_scene(scene)
			,_image(scene->_camera->imageWidth(), scene->_camera->imageHeight()) {

		}
		void step() {
			step_image(_scene, _image);
		}
		std::shared_ptr<rt::Scene> _scene;
		Image _image;
	};
}
