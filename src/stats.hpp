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

	/*
	https://qpp.bitbucket.io/post/streaming_algorithm/
	*/
	class OnlineMean {
	public:
		void addSample(double x) {
			_mean = (x - _mean) / double(_n + 1.0) + _mean;
			_n++;
		}
		double mean() const {
			return _mean;
		}
		int sampleCount() const {
			return _n;
		}
	private:
		int _n = 0;
		double _mean = 0.0;
	};

	/*
	https://qpp.bitbucket.io/post/streaming_algorithm/
	*/
	class OnlineVariance {
	public:
		void addSample(double x) {
			double mu_pre = _mean.mean();
			_mean.addSample(x);
			double mu_new = _mean.mean();
			_m += (x - mu_pre) * (x - mu_new);
		}
		double sampleVariance() const {
			return _mean.sampleCount() == 0 ? 0.0 : _m / _mean.sampleCount();
		}
		double unbiasedVariance() const {
			return _mean.sampleCount() == 0 ? 0.0 : _m / (_mean.sampleCount() + 1.0);
		}
	private:
		OnlineMean _mean;
		double _m = 0.0;
	};

	/*
	  https://softwareengineering.stackexchange.com/questions/337617/algorithm-for-windowed-online-covariance
	*/
	class OnlineCovariance {
	public:
		void addSample(double x, double y) {
			_n++;
			double dx = (x - _meanX) / _n;
			double dy = (y - _meanY) / _n;
			_meanX += dx;
			_meanY += dy;
			_C += (_n - 1.0) * dx * dy - _C / _n;
		}

		double sampleCovariance() const {
			return _n == 0 ? 0.0 : _C;
		}
		double unbiasedCovariance() const {
			return _n == 0 ? 0.0 : _n / (_n - 1) * _C;
		}
	private:
		int _n = 0;
		double _meanX = 0.0;
		double _meanY = 0.0;
		double _C = 0.0;
	};
}