#pragma once

#include <functional>

namespace rt {
	inline double Sqr(double x) {
		return x * x;
	}

	class Remap {
	public:
		Remap(double inputMin, double inputMax, double outputMin, double outputMax) {
			_a = ((outputMax - outputMin) / (inputMax - inputMin));
			_b = -inputMin * _a + outputMin;
		}
		double operator()(double x) const {
			return _a * x + _b;
		}
	private:
		double _a = 0.0;
		double _b = 0.0;
	};
	inline double remap(double value, double inputMin, double inputMax, double outputMin, double outputMax)
	{
		return (value - inputMin) * ((outputMax - outputMin) / (inputMax - inputMin)) + outputMin;
	}

	/*
	シンプソンの公式
	*/
	inline double integrate_simpson(std::function<double(double)> f, double a, double b) {
		return (b - a) / 6.0 * (f(a) + 4.0 * f((a + b) * 0.5) + f(b));
	}
	/*
	合成シンプソン公式
	nは偶数
	*/
	inline double integrate_composite_simpson(std::function<double(double)> f, double a, double b, int n) {
		assert((n & 0x1) == 0);
		double sum = 0;
		double h = (b - a) / n;
		for (int i = 1; i < n; ++i) {
			double c = (i & 0x1) ? 4.0 : 2.0;
			double x = a + h * i;
			sum += c * f(x);
		}
		sum += f(a) + f(b);
		return sum * h / 3.0;
	}
}