#pragma once

namespace rt {
	class Kahan {
	public:
		Kahan() {}
		Kahan(double value) {}

		void add(double x) {
			double y = x - _c;
			double t = _sum + y;
			_c = (t - _sum) - y;
			_sum = t;
		}
		void operator=(double x) {
			_sum = x;
			_c = 0.0;
		}
		void operator+=(double x) {
			add(x);
		}
		operator double() const {
			return _sum;
		}
	private:
		double _sum = 0.0;
		double _c = 0.0;
	};

	// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm
	class IncrementalStatatics {
	public:
		void addSample(double x) {
			_n++;
			double delta = x - _mean;
			_mean += delta / _n;
			double delta2 = x - _mean;
			_M2 += delta * delta2;
		}
		double variance() const {
			// return _M2 / (_n - 1);
			return _M2 / _n;
		}
		double avarage() const {
			return _mean;
		}

		IncrementalStatatics merge(const IncrementalStatatics &rhs) const {
			IncrementalStatatics r;

			double ma = _mean;
			double mb = rhs._mean;
			double N = _n;
			double M = rhs._n;
			double N_M = N + M;
			double a = N / N_M;
			double b = M / N_M;
			r._mean = a * ma + b * mb;
			r._M2 = _M2 + rhs._M2;
			r._n = N_M;

			return r;
		}
	private:
		Kahan _mean = 0.0;
		Kahan _M2 = 0.0;
		int _n = 0;
	};
}