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
		"[Fur Np Integral]"
	};
	Catch::Session().run(sizeof(custom_argv) / sizeof(custom_argv[0]), custom_argv);
#else
	// 全部やる場合
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
			double u = v * glm::log(glm::exp(1.0 / v) - 2.0 * eps1 * glm::sinh(1.0 / v));
			double theta_cone = -glm::asin(sinThetaO);
			double theta_tap = glm::pi<double>() * 0.5 - theta_cone;
			double theta_i = glm::asin(u * glm::cos(theta_tap) + glm::sqrt(1.0 - u * u) * glm::cos(glm::two_pi<double>() * eps2) * glm::sin(theta_tap));

			static rt::Remap toIndex(-glm::pi<double>() * 0.5, glm::pi<double>() * 0.5, 0, histgram.size());
			int index = toIndex(theta_i);
			index = glm::clamp(index, 0, (int)histgram.size());
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