#pragma once

#include <cmath>

namespace rt {
	/*
	http://muses.muses.tottori-u.ac.jp/faculty/ogata/lecture/numerical-analysis.pdf
	より
	ax^3 + bx^2 + cx + d == 0
	の最小0個 最大３個　実数解を求める。解の個数を返す
	*/
	inline int cardano_solve(double a, double b, double c, double d, double x[3]) {
		constexpr double eps = 1e-9;
		constexpr double pi = 3.14159265358979323846264338327950288;

		if (std::abs(a) < eps) {
			if (std::abs(b) < eps) {
				// ax + b == 0
				a = c;
				b = d;

				if (eps < std::abs(a)) {
					x[0] = -b / a;
					return 1;
				}
				else {
					return 0;
				}
			}

			a = b;
			b = c;
			c = d;

			// ax^2 + bx + c == 0
			// b * b - 4.0 * a * c;
			double D = std::fma(-4.0 * a, c, b * b);
			if (0.0 <= D) {
				double sqrtD = std::sqrt(D);
				double a2 = 2.0 * a;
				x[0] = (-b + sqrtD) / a2;
				x[1] = (-b - sqrtD) / a2;
				return 2;
			}
			else {
				return 0;
			}
		}

		b = b / a;
		c = c / a;
		d = d / a;

		a = b;
		b = c;
		c = d;

		// double m = (3.0 * b - a * a) * (1.0 / 3.0);
		// (3.0 * b - a * a) * (1.0 / 3.0);
		// b - a * a / 3;
		double m = std::fma(a / 3.0, -a, b);

		// double n = (2.0 * a * a * a - 9.0 * a * b + 27.0 * c) / 27.0;
		// (2.0 * a * a * a - 9.0 * a * b + 27.0 * c) / 27.0;
		// ((2.0 * a * a - 9.0 * b) * a + 27.0 * c) / 27.0;
		// (2.0 * a * a - 9.0 * b) * a / 27.0 + c)
		// (a * a - 4.5 * b) * a / 13.5 + c)
		double n = std::fma(std::fma(a, a, -4.5 * b), a / 13.5, c);

		// n * n / 4.0 + m * m * m / 27.0;
		double nn_over_4 = n * n * 0.25;
		double mmm = m * m * m;
		double mmm_over_27 = mmm / 27.0;
		double D = nn_over_4 + mmm_over_27;

		int N = 0;
		std::array<double, 3> z;
		if (std::abs(D) < eps) {
			// D == 0
			double T = std::sqrt(-m / 3.0);
			z[0] = 0.0 < n ? T : -T;
			z[1] = 0.0 < n ? -2.0 * T : 2.0 * T;
			N = 2;
		}
		else if (0.0 < D) {
			// 0 < D
			double sqrtD = std::sqrt(D);
			// double s = -n * 0.5 + sqrtD;
			// double t = -n * 0.5 - sqrtD;
			double s = std::fma(-n, 0.5, sqrtD);
			double t = std::fma(-n, 0.5, -sqrtD);
			double lhs = s < 0.0 ? -std::pow(-s, 1.0 / 3.0) : std::pow(s, 1.0 / 3.0);
			double rhs = t < 0.0 ? -std::pow(-t, 1.0 / 3.0) : std::pow(t, 1.0 / 3.0);
			z[0] = lhs + rhs;
			N = 1;
		}
		else {
			// D < 0
			// double arg = glm::sqrt((n * n / 4.0) / (- m * m * m / 27.0));
			double arg = std::sqrt(-nn_over_4 * 27.0 / mmm);
			double radian = std::acos(n > 0.0 ? -arg : arg);
			double lhs = 2.0 * std::sqrt(-m / 3.0);
			double offset = radian / 3.0;
			constexpr double step = 2.0 * pi / 3.0;
			for (int i = 0; i < 3; ++i) {
				z[i] = lhs * std::cos(offset + step * i);
			}
			N = 3;
		}

		double offset = -a / 3.0;
		for (int i = 0; i < N; ++i) {
			x[i] = z[i] + offset;
		}
		return N;
	}

	inline double cubic_evaluate(double x, double a, double b, double c, double d) {
		// a * x * x * x + b * x * x + c * x + d
		// ((a * x + b) * x + c) * x + d
		return std::fma(std::fma(std::fma(a, x, b), x, c), x, d);
	}
	inline double derivative_cubic_evaluate(double x, double a, double b, double c) {
		// 3.0 * a * x * x + 2.0 * b * x + c
		// (3.0 * a * x + 2.0 * b) * x + c
		return std::fma(std::fma(3.0 * a, x, 2.0 * b), x, c);
	}

	inline int cardano_solve_and_newton(double a, double b, double c, double d, double x[3]) {
		int N = cardano_solve(a, b, c, d, x);
		for (int j = 0; j < N; ++j) {
			double t = x[j];
			for (int i = 0; i < 2; ++i) {
				double y = cubic_evaluate(t, a, b, c, d);
				double dydx = derivative_cubic_evaluate(t, a, b, c);
				t = t - y / dydx;
			}
			x[j] = t;
		}
		return N;
	}
}