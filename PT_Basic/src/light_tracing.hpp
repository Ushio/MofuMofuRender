#pragma once

#include <tbb/tbb.h>
#include "scene.hpp"

#define LIGHT_TRACING 1

namespace rt {
	template<typename T, typename BinaryOp>
	T atomicOp(std::atomic<T> &f, T d, BinaryOp _Op)
	{
		using namespace std;

		auto old = f.load();
		auto desired = _Op(old, d);
		while (!f.compare_exchange_weak(old, desired)) {
			desired = _Op(old, d);
		}
		return desired;
	}

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

		bool isInclude(int x, int y) const {
			if (x < 0 || _w <= x) {
				return false;
			}
			if (y < 0 || _h <= y) {
				return false;
			}
			return true;
		}

		void add(int x, int y, Vec3 c) {
			int index = y * _w + x;
			for (int i = 0; i < 3; ++i) {
				atomicOp(_pixels[index].color[i], c[i], [](double a, double b) { return a + b; });
			}
			_pixels[index].sample++;
		}

		struct Pixel {
			Pixel() {
				sample = 0;
				for (int i = 0; i < 3; ++i) {
					color[i] = 0.0;
				}
			}
			Vec3 toColorVector() const {
				return Vec3(color[0].load(), color[1].load(), color[2].load());
			}
			std::atomic<int> sample;
			std::atomic<double> color[3];
			Xor random;
		};
		const Pixel *pixel(int x, int y) const {
			return _pixels.data() + y * _w + x;
		}
		Pixel *pixel(int x, int y) {
			return _pixels.data() + y * _w + x;
		}

		void addSampleGlobal(int n) {
			_sampleGlobal += n;
		}
		int sampleGlobal() const {
			return _sampleGlobal;
		}
	private:
		int _w = 0;
		int _h = 0;
		std::vector<Pixel> _pixels;

		int _sampleGlobal = 0;
	};

	inline Vec3 radiance(std::shared_ptr<rt::Scene> scene, PeseudoRandom *random, Image *image, int x, int y) {
#if LIGHT_TRACING
		rt::Vec3 light_p;
		rt::Vec3 light_n;
		rt::Vec3 Le;
		scene->sampleEmissive(&light_p, &light_n, &Le, random);
		double p_area = 1.0 / scene->emissiveArea();
		auto camera = scene->camera();

		double pdf = p_area;
		rt::Vec3 n_t = light_n;
		rt::Vec3 x_t = light_p;
		rt::Vec3 T = rt::Vec3(1.0);

		for (int i = 0; i < 10; ++i) {
			auto x_t_plus_1 = scene->camera()->sampleLens(random);
			Vec3 x1_to_x0 = x_t_plus_1 - x_t;
			Vec3 x0_to_x1 = -x1_to_x0;

			if (scene->visible(x_t, x_t_plus_1)) {
				// 裏面からは見えない
				// うらからはfindで弾ける・・・と思う
				if (glm::dot(x1_to_x0, n_t) > 0.0) {
					double G = glm::dot(camera->front(), glm::normalize(x0_to_x1)) * glm::dot(n_t, glm::normalize(x1_to_x0)) / glm::length2(x0_to_x1);
					double Wi = camera->Wi(x_t_plus_1, x_t, n_t);
					double PDF = pdf * camera->lensPDF();
					Vec3 contrib = Wi * Le * T * G / PDF;

					int sampleX;
					int sampleY;
					if (camera->findPixel(x_t, glm::normalize(x1_to_x0), &sampleX, &sampleY)) {
						image->add(sampleX, sampleY, contrib);
					}
				}
			}
			
			Vec3 sample = sample_cosine_weighted_hemisphere_brdf(random);
			auto wo = from_bxdf(n_t, sample);
			double pdf_sr = cosine_weighted_hemisphere_pdf_brdf(sample);

			Material mat;
			Intersection intersection;
			double tmin = std::numeric_limits<double>::max();
			Ray ray;
			ray.o = x_t + wo * 0.000001;
			ray.d = wo;
			if (scene->intersect(ray, &mat, &intersection, &tmin)) {
				auto x_t_plus_1 = ray.o + ray.d * tmin;
				auto n_t_plus_1 = intersection.normal;
				double G = glm::dot(n_t, wo) * glm::dot(-wo, n_t_plus_1) / (tmin * tmin);

				x_t = x_t_plus_1;
				n_t = n_t_plus_1;

				if (mat.is<LambertianMaterial>()) {
					auto material = mat.get<LambertianMaterial>();
					auto brdf = material.R * Vec3(glm::one_over_pi<double>());
					T *= brdf * G;
				} else {
					// Not work yet..
				}

				double pdf_area = pdf_sr * glm::dot(n_t, -wo) / (tmin * tmin);
				pdf *= pdf_area;
			}
			else {
				break;
			}
			// 
		}

		//rt::Vec3 d;
		//{
		//	Vec3 sample = sample_cosine_weighted_hemisphere_brdf(random);
		//	double pdf_omega = cosine_weighted_hemisphere_pdf_brdf(sample);
		//	auto wi = from_bxdf(light_n, sample);
		//}
		//Ray ray(light_p, d);

		//auto sampleLens = scene->camera()->sampleLens(random);
		//if (scene->visible(light_p, sampleLens)) {
		//	Vec3 x1_to_x0 = sampleLens - light_p;
		//	Vec3 x0_to_x1 = -x1_to_x0;

		//	if (glm::dot(x1_to_x0, light_n) > 0.0) {
		//		double G = glm::dot(camera->front(), glm::normalize(x0_to_x1)) * glm::dot(light_n, glm::normalize(x1_to_x0)) / glm::length2(x0_to_x1);
		//		double Wi = camera->Wi(sampleLens, light_p, light_n);
		//		double PDF = p_area * camera->lensPDF();
		//		Vec3 contrib = Wi * Le * G / PDF;

		//		int sampleX;
		//		int sampleY;
		//		if (camera->findPixel(light_p, glm::normalize(sampleLens - light_p), &sampleX, &sampleY)) {
		//			image->add(sampleX, sampleY, contrib);
		//		}
		//	}
		//}

		return Vec3();
#else
		Ray ray;
		scene->camera()->sampleRay(random, x, y, &ray.o, &ray.d);

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
			}
			else {
				break;
			}
		}
		return Lo;
#endif
	}


	inline void step_image(std::shared_ptr<rt::Scene> scene, Image &image) {
		auto camera = scene->camera();
		tbb::parallel_for(tbb::blocked_range<int>(0, camera->imageHeight()), [&](const tbb::blocked_range<int> &range) {
			for (int y = range.begin(); y < range.end(); ++y) {
				for (int x = 0; x < camera->imageWidth(); ++x) {
					PeseudoRandom *random = &image.pixel(x, y)->random;
#if LIGHT_TRACING
					radiance(scene, random, &image, x, y);
#else
					image.add(x, y, radiance(scene, random, &image, x, y));
#endif
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
			_image.addSampleGlobal(_image.width() * _image.height());
			// _image.addSampleGlobal(1);
		}
		std::shared_ptr<rt::Scene> _scene;
		Image _image;
	};
}
