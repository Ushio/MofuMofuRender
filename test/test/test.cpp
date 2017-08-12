// test.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#pragma warning(disable:4996)

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "bezier.hpp"
#include "scene.hpp"
#include "peseudo_random.hpp"

TEST_CASE("Hello", "[ray_projection]") {
	rt::Xor random;
	for (int i = 0; i < 1000; ++i) {
		rt::Vec3 d = rt::uniform_on_unit_sphere(&random);
		rt::Vec3 o = rt::Vec3(random.uniform(-10.0, 10.0), random.uniform(-10.0, 10.0), random.uniform(-10.0, 10.0));
		auto pj = rt::ray_projection(o, d);

		rt::Vec3 origin = pj * o;
		rt::Vec3 origin_add_d = pj * (o + d);
		REQUIRE(false);
		REQUIRE(glm::length(origin) < 0.0000001);
		REQUIRE(glm::distance(origin_add_d, rt::Vec3(0.0, 0.0, 1.0)) < 0.0000001);
	}
}
