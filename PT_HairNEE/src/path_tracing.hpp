#pragma once

#include <tbb/tbb.h>
#include "scene.hpp"
#include "misc.hpp"
#include <fixed_size_function.hpp>

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


	class IBL {
	public:
		IBL() {}
		IBL(int w, int h) :_w(w), _h(h), _pixels(h*w) {

		}
		int width() const {
			return _w;
		}
		int height() const {
			return _h;
		}

		Vec3 radiance(Vec3 wi) const {
			double phi = atan2(wi.x, wi.z);
			double cosTheta = wi.y;
			double theta = acos(cosTheta);
			static Remap toX(-glm::pi<double>(), glm::pi<double>(), 0, _w);
			static Remap toY(0.0, glm::pi<double>(), 0, _h);
			int x = toX(phi);
			int y = toY(theta);
			x = glm::clamp(0, x, _w - 1);
			y = glm::clamp(0, y, _h - 1);
			return *pixel(x, y);
		}

		const Vec3 *pixel(int x, int y) const {
			return _pixels.data() + y * _w + x;
		}
		Vec3 *pixel(int x, int y) {
			return _pixels.data() + y * _w + x;
		}
	private:
		int _w = 0;
		int _h = 0;
		std::vector<Vec3> _pixels;
	};

	inline Vec3 radiance(std::shared_ptr<rt::Scene> scene, Ray ray, PeseudoRandom *random, const IBL &ibl) {
		// 光源はLambertianMaterialのみであるので、パラメータはこれだけでOK
		fixed_size_function<double(Vec3, Vec3), 128> evaluate_mis_weight_implicit = [](Vec3 light_p, Vec3 light_n) {
			return 1.0;
		};

		Vec3 Lo;
		Vec3 T(1.0);
		const int kDepth = 10;
		for (int i = 0; i < kDepth; ++i) {
			bool isLast = i + 1 == kDepth;
			Material mat;
			Intersection intersection;
			double tmin = std::numeric_limits<double>::max();

			if (scene->intersect(ray, &mat, &intersection, &tmin)) {
				// NEE のロシアンルーレット
				// より短い経路に計算を集中させるべきだ
				auto connection_russian_roulette = [](double x) {
					// https://www.desmos.com/calculator/iacvjvy9ic
					double a = 0.05;
					double b = 0.3;
					double c = 1.6;
					return a + (-tanh(b * x - c) + 1.0) * 0.5 * (1.0 - a);
				};
				double russian_roulette_continue = connection_russian_roulette(i);
				// |             continue                |      reject     |
				// |           e.g. 80 %                 |  e.g. 20 %      | 
				bool doNEE = true;
				if (russian_roulette_continue < random->uniform()) {
					doNEE = false;
				}

				/*
				russian_roulette_continue = 1.0;
				doNEE = true;
				*/

				if(mat.is<LambertianMaterial>()) {
					auto material = mat.get<LambertianMaterial>();
					Vec3 px = ray.o + ray.d * tmin;

					if(doNEE) {
						Vec3 p;
						Vec3 n;
						Vec3 emissive;
						scene->sampleEmissive(&p, &n, &emissive, random);

						double area = scene->emissiveArea();
						double p_A_explicit = 1.0 / area;

						Vec3 wi = glm::normalize(p - px);

						auto brdf = material.R * Vec3(glm::one_over_pi<double>());

						double cosTheta0 = glm::max(glm::dot(intersection.normal, wi), 0.0);
						double cosTheta1 = glm::max(glm::dot(n, -wi), 0.0);
						double G = cosTheta0 * cosTheta1 / glm::distance2(px, p);

						double p_omega_implicit = cosine_weighted_hemisphere_pdf_brdf(to_bxdf_basis_transform(intersection.normal) * wi);
						double p_A_implicit = p_omega_implicit * cosTheta1 / glm::distance2(px, p);

						// バランスヒューリスティック
						// double this_mis_weight = p_A_explicit / (p_A_implicit + p_A_explicit);
						
						// パワーヒューリスティック
						// 次のimplicitはサンプルしないため、その場合のMIS Weight は 1.0
						double this_mis_weight = isLast ? 1.0 : p_A_explicit * p_A_explicit / (p_A_implicit * p_A_implicit + p_A_explicit * p_A_explicit);

						if (glm::epsilon<double>() < G) {
							if (scene->visible(px, p)) {
								Lo += this_mis_weight * T * brdf * G * emissive / p_A_explicit / russian_roulette_continue;
								// Lo += T * brdf * G * emissive / p_A_explicit;
							}
						}
					}

					Lo += evaluate_mis_weight_implicit(px, intersection.normal) * material.Le * T;
					// Lo += material.Le * T;

					Vec3 sample = sample_cosine_weighted_hemisphere_brdf(random);
					double p_omega = cosine_weighted_hemisphere_pdf_brdf(sample);
					auto wi = from_bxdf(intersection.normal, sample);

					auto brdf = material.R * Vec3(glm::one_over_pi<double>());
					auto cos_term = abs_cos_theta_bxdf(sample);

					T *= brdf * cos_term / p_omega;
					ray = Ray(px + wi * 0.000001, wi);

					evaluate_mis_weight_implicit = [=](Vec3 light_p, Vec3 light_n) {
						double cosTheta1 = glm::abs(glm::dot(light_n, -wi));
						double p_A_implicit = p_omega * cosTheta1 / glm::distance2(px, light_p);
						double p_A_explicit = 1.0 / scene->emissiveArea();

						// バランスヒューリスティック
						// return p_A_implicit / (p_A_implicit + p_A_explicit);

						// パワーヒューリスティック
						return p_A_implicit * p_A_implicit / (p_A_implicit * p_A_implicit + p_A_explicit * p_A_explicit);
					};
				} 
				else if (mat.is<FurMaterial>()) {
					auto material = mat.get<FurMaterial>();
					Vec3 wo = -ray.d;
					Vec3 px = ray.o + ray.d * tmin;

					if (doNEE) {
						Vec3 p;
						Vec3 n;
						Vec3 emissive;
						scene->sampleEmissive(&p, &n, &emissive, random);
						double area = scene->emissiveArea();
						double p_A_explicit = 1.0 / area;

						Vec3 wi = glm::normalize(p - px);

						auto to_bsdf = to_fur_bsdf_basis_transform(material.tangent);

						// アイ方向
						Vec3 bsdf_wo = to_bsdf * wo;

						// ライト方向
						Vec3 bsdf_wi = to_bsdf * wi;

						Vec3 bsdf_value = fur_bsdf(bsdf_wi, bsdf_wo, material.params);

						double cosTheta0 = AbsCosThetaForFur(bsdf_wi);
						double cosTheta1 = glm::max(glm::dot(n, -wi), 0.0);
						double G = cosTheta0 * cosTheta1 / glm::distance2(px, p);

						double p_omega_implicit = pdfFur(bsdf_wi, bsdf_wo, material.params);
						double p_A_implicit = p_omega_implicit * cosTheta1 / glm::distance2(px, p);
						// バランスヒューリスティック
						// double this_mis_weight = p_A_explicit / (p_A_implicit + p_A_explicit);
						// パワーヒューリスティック
						// 次のimplicitはサンプルしないため、その場合のMIS Weight は 1.0
						double this_mis_weight = isLast ? 1.0 : p_A_explicit * p_A_explicit / (p_A_implicit * p_A_implicit + p_A_explicit * p_A_explicit);

						if (glm::epsilon<double>() < G) {
							if (scene->visible(px, p)) {
								Lo += this_mis_weight * T * bsdf_value * G * emissive / p_A_explicit;
							}
						}
					}

					double p_omega;
					Vec3 wi = sampleFur(
					    { random->uniform(), random->uniform(), random->uniform(), random->uniform() },
						wo, material.params, &p_omega
					);

					auto to_bsdf = to_fur_bsdf_basis_transform(material.tangent);

					// アイ方向
					Vec3 bsdf_wo = to_bsdf * wo;

					// ライト方向
					Vec3 bsdf_wi = to_bsdf * wi;

					Vec3 bsdf_value = fur_bsdf(bsdf_wi, bsdf_wo, material.params);
					T *= (bsdf_value / p_omega) * AbsCosThetaForFur(bsdf_wi);
					ray = Ray(px + wi * 0.000001, wi);

					evaluate_mis_weight_implicit = [=](Vec3 light_p, Vec3 light_n) {
						double cosTheta1 = glm::abs(glm::dot(light_n, -wi));
						double p_A_implicit = p_omega * cosTheta1 / glm::distance2(px, light_p);
						double p_A_explicit = 1.0 / scene->emissiveArea();

						// バランスヒューリスティック
						// return p_A_implicit / (p_A_implicit + p_A_explicit);

						// パワーヒューリスティック
						return p_A_implicit * p_A_implicit / (p_A_implicit * p_A_implicit + p_A_explicit * p_A_explicit);
					};
				} else if (mat.is<SpecularMaterial>()) {
					Vec3 wi = glm::reflect(ray.d, intersection.normal);
					ray = Ray(ray.o + ray.d * tmin + wi * 0.000001, wi);

					evaluate_mis_weight_implicit = [](Vec3 light_p, Vec3 light_n) {
						return 1.0;
					};
				}
			}
			else {
				// IBL
				// IBLをサンプルする戦略は今回は暗黙の寄与のみであり、そのためMIS Weightは 1
				Lo += ibl.radiance(ray.d) * T;
				break;
			}
		}
		return Lo;
	}

	inline void step_image(std::shared_ptr<rt::Scene> scene, Image &image, const IBL &ibl) {
		auto camera = scene->camera();
		tbb::parallel_for(tbb::blocked_range<int>(0, camera->imageHeight()), [&](const tbb::blocked_range<int> &range) {
			for (int y = range.begin(); y < range.end(); ++y) {
				for (int x = 0; x < camera->imageWidth(); ++x) {
					PeseudoRandom *random = &image.pixel(x, y)->random;
					double dx = random->uniform(-0.5, 0.5);
					double dy = random->uniform(-0.5, 0.5);
					// Ray ray = camera->generateRay(x + dx, y + dy);

					auto camera = scene->_camera;
					auto filmP = camera->filmToWorld(x + dx, y + dy);
					auto lensP = camera->sampleLens(random);
					auto lenspA = 1.0 / camera->lensArea();

					auto toFocal = glm::normalize(camera->origin() - filmP);
					auto toFocalStep = glm::dot(toFocal, camera->front());
					auto focalTmin = camera->_setting._focalLength / toFocalStep;
					auto focalPoint = camera->origin() + toFocal * focalTmin;
					auto filmToLens = lensP - filmP;

					Ray ray(lensP, glm::normalize(focalPoint - lensP));

					auto L = radiance(scene, ray, random, ibl);
					// 念のために浮動小数点チェックとクランプ
					if (glm::all(glm::isfinite(L))) {
						L = glm::clamp(L, Vec3(0.0), Vec3(1000.0));
						image.add(x, y, L);
					}
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
			step_image(_scene, _image, _ibl);
			_spp++;
		}
		int spp() const {
			return _spp;
		}
		std::shared_ptr<rt::Scene> _scene;
		Image _image;
		IBL _ibl;
		int _spp = 0;
	};
}
