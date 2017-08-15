// test.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#pragma warning(disable:4996)

#define CATCH_CONFIG_RUNNER
#include "catch.hpp"

#include "bezier.hpp"
#include "scene.hpp"
#include "peseudo_random.hpp"
#include "misc.hpp"

int main(int argc, char* const argv[])
{
#if 1
	// テストを指定する場合
	char* const custom_argv[] = {
		"",
		"[Fur PDF]"
	};
	Catch::Session().run(sizeof(custom_argv) / sizeof(custom_argv[0]), custom_argv);
#else
	// 全部やる場合
	char* const custom_argv[] = {
		"",
	};
	Catch::Session().run(sizeof(custom_argv) / sizeof(custom_argv[0]), custom_argv);
#endif

	std::cin.get();
	return 0;
}

TEST_CASE("Simpson", "[Simpson]") {
	// https://www.wolframalpha.com/input/?i=integrate+sin(x)%2Bcos(3x)%2F2
	auto f = [](double x) {
		return glm::sin(x) + glm::cos(3.0 * x) * 0.5;
	};
	auto int_f_range = [](double a, double b) {
		auto int_f = [](double x) {
			return (glm::sin(3.0 * x) - 6.0 * glm::cos(x)) / 6.0;
		};
		return int_f(b) - int_f(a);
	};

	rt::Xor random;
	for (int i = 0; i < 100000; ++i) {
		double a = random.uniform(-10.0, 10.0);
		double b = a + random.uniform(-0.3, 0.3);
		double analytical = int_f_range(a, b);
		double simpson = rt::integrate_simpson(f, a, b);
		double composite_simpson = rt::integrate_composite_simpson(f, a, b, 10);
		REQUIRE(glm::abs(analytical - simpson) < 0.0001);
		REQUIRE(glm::abs(analytical - composite_simpson) < 0.0001);
	}
}

TEST_CASE("Bezier Collision", "[Bezier Collision]") {
	rt::Xor random;
	for (int i = 0; i < 1000; ++i) {
		rt::Vec3 d = rt::uniform_on_unit_sphere(&random);
		rt::Vec3 o = rt::Vec3(random.uniform(-10.0, 10.0), random.uniform(-10.0, 10.0), random.uniform(-10.0, 10.0));
		auto pj = rt::ray_projection(o, d);

		rt::Vec3 origin = pj * o;
		rt::Vec3 origin_add_d = pj * (o + d);
		
		REQUIRE(glm::length(origin) < 0.0000001);
		REQUIRE(glm::distance(origin_add_d, rt::Vec3(0.0, 0.0, 1.0)) < 0.0000001);
	}
}

TEST_CASE("Fur Mp Integration", "[Fur Mp Integration]") {
	// Mpが単体でエネルギー保存、確立密度関数が満たすべき性質を満たしている
	// \int _{ -\frac { \pi  }{ 2 }  }^{ \frac { \pi  }{ 2 }  }{ M_{ p }\left( \theta _{ o },\theta _{ i } \right) \cos { \theta _{ i } } d\theta _{ i } } =1
	rt::Xor random;
	for (int j = 0; j < 1000; ++j) {
		double theta_o = random.uniform(-glm::pi<double>() * 0.5, glm::pi<double>() * 0.5);
		double cosThetaO = glm::cos(theta_o);
		double sinThetaO = glm::sin(theta_o);
		double v = random.uniform(0.001, 10.0);

		double integral = rt::integrate_composite_simpson([&](double theta_i) {
			double cosThetaI = glm::cos(theta_i);
			double sinThetaI = glm::sin(theta_i);
			double mp = rt::Mp(sinThetaI, cosThetaI, sinThetaO, cosThetaO, v);
			return mp * cosThetaI;
		}, -glm::pi<double>() * 0.5, glm::pi<double>() * 0.5, 1000);
		REQUIRE(glm::abs(integral - 1.0) < 0.02);
	}
}

/*
このテストはビジュアライズしたほうが遥かにいい
*/
TEST_CASE("Fur Mp Importance sampling", "[Fur Mp Importance sampling]") {
	// サンプリングが確立密度関数に従っているか
	rt::Xor random;
	for (int j = 0; j < 100; ++j) {
		double theta_o = random.uniform(-glm::pi<double>() * 0.5, glm::pi<double>() * 0.5);
		double cosThetaO = glm::cos(theta_o);
		double sinThetaO = glm::sin(theta_o);
		double v = random.uniform(0.01, 10.0);
		std::array<int, 50> histgram = {};

		int NSample = 1000000;
		for (int i = 0; i < NSample; ++i) {
			double eps1 = random.uniform();
			double eps2 = random.uniform();
			double theta_i = rt::sampleMp(eps1, eps2, v, sinThetaO);
			static rt::Remap toIndex(-glm::pi<double>() * 0.5, glm::pi<double>() * 0.5, 0, histgram.size());
			int index = toIndex(theta_i);
			index = glm::clamp(index, 0, (int)histgram.size() - 1);
			histgram[index]++;
		}

		double deltaTheta = glm::pi<double>() / histgram.size();
		for (int i = 0; i < histgram.size(); ++i) {
			static rt::Remap toTheta(0, histgram.size(), -glm::pi<double>() * 0.5, glm::pi<double>() * 0.5);
			double theta_i = toTheta(i + 0.5);
			double P = (double)histgram[i] / (double)NSample;
			double similarMp = P / (deltaTheta * glm::cos(theta_i));

			double cosThetaI = glm::cos(theta_i);
			double sinThetaI = glm::sin(theta_i);

			double mp = rt::Mp(sinThetaI, cosThetaI, sinThetaO, cosThetaO, v);

			REQUIRE(glm::abs(mp - similarMp) < 0.1);
		}
	}
}

TEST_CASE("Fur TrimmedLogistic", "[Fur TrimmedLogistic]") {
	// 積分が 1 になるべき
	rt::Xor random;
	for (int j = 0; j < 1000; ++j) {
		double s = random.uniform(0.01, 2.0);

		double a = random.uniform(-5.0, 0.0);
		double b = random.uniform(0.0, 5.0);
		if (a > b) {
			std::swap(a, b);
		}
		double integral = rt::integrate_composite_simpson([&](double x) {
			double va = rt::TrimmedLogistic(x, s, a, b);
			return va;
		}, a, b, 1000);
		REQUIRE(glm::abs(integral - 1.0) < 0.001);
	}
}

TEST_CASE("Fur Np Integral", "[Fur Np Integral]") {
	using namespace rt;

	// 積分が 1 になるべき
	rt::Xor random;
	for (int j = 0; j < 1000; ++j) {
		double thetaO = random.uniform(-glm::pi<double>() * 0.5, glm::pi<double>() * 0.5);
		double sinThetaO = glm::sin(thetaO);
		double cosThetaO = glm::cos(thetaO);
		double betaN = random.uniform(0.01, 1.0);
		double s = betan_to_s(betaN);
		double eta = 1.55;
		double phiO = random.uniform(-glm::pi<double>(), glm::pi<double>());
		double h = random.uniform(-1.0, 1.0);
		int p = random.generate() % 10;

		double integral = integrate_composite_simpson([&](double phiI) {
			double phi = phiI - phiO;
			double gammaO = SafeASin(h);
			double etap = glm::sqrt(eta * eta - Sqr(sinThetaO)) / cosThetaO;
			double sinGammaT = h / etap;
			double cosGammaT = SafeSqrt(1 - Sqr(sinGammaT));
			double gammaT = SafeASin(sinGammaT);
			double np = Np(phi, p, s, gammaO, gammaT);
			return np;
		}, -glm::pi<double>(), glm::pi<double>(), 5000);

		REQUIRE(glm::abs(integral - 1.0) < 0.001);
	}
}

TEST_CASE("Np Logistic Sample", "[Np Logistic Sample]") {
	rt::Xor random;
	for (int j = 0; j < 100; ++j) {
		double s = random.uniform(0.1, 2.0);

		double a = -glm::pi<double>();
		double b = glm::pi<double>();

		std::array<int, 100> histgram = {};
		int NSample = 1000000;
		for (int i = 0; i < NSample; ++i) {
			static rt::Remap toIndex(a, b, 0, histgram.size());

			double sample = rt::sampleTrimmedLogistic(random.uniform(), s, a, b);
			int index = toIndex(sample);
			index = glm::clamp(index, 0, (int)histgram.size() - 1);
			histgram[index]++;
		}

		for (int i = 0; i < histgram.size(); ++i) {
			double p = (double)histgram[i] / NSample;
			double similarTrimmedLogistic = p / ((b - a) / histgram.size());
			static rt::Remap toX(0, histgram.size(), a, b);
			double x = toX(i + 0.5);
			double trimmedLogistic = rt::TrimmedLogistic(x, s, a, b);
			REQUIRE(glm::abs(trimmedLogistic - similarTrimmedLogistic) < 0.05);
		}
	}
}


/*
 このテストはビジュアライズしたほうが遥かにいい
*/
TEST_CASE("Np Sampling", "[Np Sampling]") {
	using namespace rt;

	rt::Xor random;
	for (int j = 0; j < 100; ++j) {
		double s = betan_to_s(random.uniform(0.2, 1.0));
		double a = -glm::pi<double>();
		double b = glm::pi<double>();

		double thetaO = random.uniform(-glm::pi<double>() * 0.5, glm::pi<double>() * 0.5);
		double sinThetaO = glm::sin(thetaO);
		double cosThetaO = glm::cos(thetaO);
		double eta = 1.55;
		double h = random.uniform(-1.0, 1.0);
		int p = random.generate() % 4;
		
		double phiO = random.uniform(-glm::pi<double>(), glm::pi<double>());

		std::array<int, 100> histgram = {};
		int NSample = 1000000;
		for (int i = 0; i < NSample; ++i) {
			static rt::Remap toIndex(a, b, 0, histgram.size());

			double gammaO = SafeASin(h);
			double etap = glm::sqrt(eta * eta - Sqr(sinThetaO)) / cosThetaO;
			double sinGammaT = h / etap;
			double cosGammaT = SafeSqrt(1 - Sqr(sinGammaT));
			double gammaT = SafeASin(sinGammaT);

			double phiI = rt::sampleNp(random.uniform(), phiO, p, s, gammaO, gammaT);

			int index = toIndex(phiI);
			index = glm::clamp(index, 0, (int)histgram.size() - 1);
			histgram[index]++;
		}

		for (int i = 0; i < histgram.size(); ++i) {
			double P = (double)histgram[i] / NSample;
			double similarNp = P / ((b - a) / histgram.size());
			static rt::Remap toPhiI(0, histgram.size(), a, b);
			double phiI = toPhiI(i + 0.5);
			double phi = phiI - phiO;

			double gammaO = SafeASin(h);
			double etap = glm::sqrt(eta * eta - Sqr(sinThetaO)) / cosThetaO;
			double sinGammaT = h / etap;
			double cosGammaT = SafeSqrt(1 - Sqr(sinGammaT));
			double gammaT = SafeASin(sinGammaT);

			double np = Np(phi, p, s, gammaO, gammaT);
			REQUIRE(glm::abs(np - similarNp) < 0.1);
		}
	}
}

TEST_CASE("Ap Sampling", "[Ap Sampling]") {
	using namespace rt;

	rt::Xor random;

	for (int j = 0; j < 1000; ++j) {
		std::array<double, pMax + 1> ap_tap = {
			random.uniform(0.0, 1.0),
			random.uniform(0.0, 1.0),
			random.uniform(0.0, 1.0),
			random.uniform(0.0, 1.0)
		};
		std::array<double, pMax + 1> apPDF = toApPDF(ap_tap);
		std::array<double, pMax + 1> apCDF = toApCDF(apPDF);
		
		std::array<int, pMax + 1> histgram = {};
		int NSample = 100000;
		for (int i = 0; i < NSample; ++i) {
			int p = sampleAp(random.uniform(), apCDF);
			histgram[p]++;
		}

		for (int i = 0; i < histgram.size(); ++i) {
			double P = (double)histgram[i] / NSample;
			double PDF = apPDF[i];
			REQUIRE(glm::abs(P - PDF) < 0.01);
		}
	}
}


TEST_CASE("Fur energy conservation", "[Fur energy conservation]") {
	using namespace rt;

	rt::Xor random;
	for (int j = 0; j < 50; ++j) {
		Vec3 wo = uniform_on_unit_sphere(&random);

		Vec3 sum;
		int count = 100000;
		for (int i = 0; i < count; ++i) {
			FurBSDFParams params;
			params.h = random.uniform(-1.0, 1.0);
			params.sigma_a = Vec3(0.0);
			params.beta_n = random.uniform(0.6, 1.0);
			params.beta_m = random.uniform(0.6, 1.0);
			params.eta = 1.55;
			params.alpha = 0.0;

			double p_omega = 1.0 / (4.0 * glm::pi<double>());
			Vec3 sigma_a;
			Vec3 wi = uniform_on_unit_sphere(&random);

			sum += rt::fur_bsdf(wi, wo, params) * rt::AbsCosThetaForFur(wi) / p_omega;
		}

		double avg = sum.y / count;
		REQUIRE(glm::abs(avg - 1.0) < 0.05);
	}
}

TEST_CASE("Fur Importance Sampling", "[Fur Importance Sampling]") {
	using namespace rt;

	rt::Xor random;
	for (int j = 0; j < 50; ++j) {
		Vec3 wo = uniform_on_unit_sphere(&random);

		Vec3 sum;
		int count = 100000;
		for (int i = 0; i < count; ++i) {
			// TODO beta_m が小さいとき
			FurBSDFParams params;
			params.h = random.uniform(-1.0, 1.0);
			params.sigma_a = Vec3(0.0);
			params.beta_n = random.uniform(0.2, 1.0);
			params.beta_m = random.uniform(0.2, 1.0);
			params.eta = 1.55;
			params.alpha = 0.0;

			double p_omega;
			Vec3 sigma_a;
			Vec3 wi = sampleFur(
				{ random.uniform(), random.uniform(), random.uniform(), random.uniform()},
				wo,
				params,
				&p_omega
			);

			sum += rt::fur_bsdf(wi, wo, params) * rt::AbsCosThetaForFur(wi) / p_omega;
		}

		double avg = sum.y / count;
		REQUIRE(glm::abs(avg - 1.0) < 0.05);
	}
}

TEST_CASE("Fur PDF", "[Fur PDF]") {
	using namespace rt;

	rt::Xor random;
	for (int j = 0; j < 10000; ++j) {
		Vec3 wo = uniform_on_unit_sphere(&random);

		FurBSDFParams params;
		params.h = random.uniform(-1.0, 1.0);
		params.sigma_a = Vec3(0.0);
		params.beta_n = random.uniform(0.1, 1.0);
		params.beta_m = random.uniform(0.1, 1.0);
		params.eta = 1.55;
		params.alpha = 0.0;

		double p_omega;
		Vec3 sigma_a;
		Vec3 wi = sampleFur(
		{ random.uniform(), random.uniform(), random.uniform(), random.uniform() },
			wo,
			params,
			&p_omega
		);

		double p_omega_with_wi = pdfFur(wi, wo, params);
		REQUIRE(glm::abs(p_omega_with_wi - p_omega) < 0.0001);
	}
}