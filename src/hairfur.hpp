#pragma once

#include <math.h>
#include <array>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include "geometry.hpp"

namespace rt {
	template <int n>
	static double Pow(double v) {
		static_assert(n > 0, "Power can’t be negative");
		double n2 = Pow<n / 2>(v);
		return n2 * n2 * Pow<n & 1>(v);
	}
	template <> double Pow<1>(double v) { return v; }
	template <> double Pow<0>(double v) { return 1.0; }

	inline double Sqr(double x) {
		return x * x;
	}

	inline double SafeASin(double x) {
		if ((x >= -1.0001 && x <= 1.0001) == false) {
			printf("SafeASin warning x: %.10f\n", x);
		}
		return std::asin(glm::clamp(x, -1.0, 1.0));
	}
	inline double SafeSqrt(double x) {
		if (x < -1e-4) {
			printf("SafeSqrt warning x: %.10f\n", x);
		}
		return std::sqrt(std::max(double(0.0), x));
	}

	enum {
		pMax = 3
	};

	inline double I0(double x, int n = 10) {
		double sum = 0.0;
		double fact_m = 1.0;
		double x_over_2_pow_2m = 1.0;
		const double x_over_2 = x * 0.5;
		for (int m = 0; m < n; ++m) {
			if (m != 0) {
				fact_m *= m;
			}

			sum += 1.0 / (fact_m * fact_m) * x_over_2_pow_2m;

			x_over_2_pow_2m *= x_over_2 * x_over_2;
		}

		return sum;
	}

	inline double LogI0(double x) {
		if (x > 12.0) {
			return x + 0.5 * (-std::log(2.0 * glm::pi<double>()) + std::log(1.0 / x) + 1.0 / (8.0 * x));
		}
		return std::log(I0(x));
	}

	inline double Mp(double sinThetaI, double cosThetaI, double sinThetaO, double cosThetaO, double v) {
		double a = cosThetaI * cosThetaO / v;
		double b = sinThetaI * sinThetaO / v;
		double mp = (v <= 0.1) ?
			(glm::exp(LogI0(a) - b - 1.0 / v + 0.6931 + glm::log(1.0 / (2.0 * v)))) :
			(glm::exp(-b) * I0(a)) / (glm::sinh(1.0 / v) * 2.0 * v);
		return mp;
	}

	inline double FrDielectric(double cosThetaI, double etaI, double etaT) {
		cosThetaI = glm::clamp(cosThetaI, -1.0, 1.0);
		bool entering = cosThetaI > 0.f;
		if (!entering) {
			std::swap(etaI, etaT);
			cosThetaI = std::abs(cosThetaI);
		}

		// Compute _cosThetaT_ using Snell's law
		double sinThetaI = std::sqrt(std::max(0.0, 1.0 - cosThetaI * cosThetaI));
		double sinThetaT = etaI / etaT * sinThetaI;

		// Handle total internal reflection
		if (sinThetaT >= 1.0) return 1;
		double cosThetaT = std::sqrt(std::max(0.0, 1.0 - sinThetaT * sinThetaT));
		double Rparl = ((etaT * cosThetaI) - (etaI * cosThetaT)) / ((etaT * cosThetaI) + (etaI * cosThetaT));
		double Rperp = ((etaI * cosThetaI) - (etaT * cosThetaT)) / ((etaI * cosThetaI) + (etaT * cosThetaT));
		return (Rparl * Rparl + Rperp * Rperp) * 0.5;
	}
	inline std::array<Vec3, pMax + 1> Ap(double cosThetaO, double eta, double h, const Vec3 &T) {
		std::array<Vec3, pMax + 1> ap;
		double cosGammaO = SafeSqrt(1.0 - h * h);
		double cosTheta = cosThetaO * cosGammaO;
		double f = FrDielectric(cosTheta, 1.0, eta);
		ap[0] = Vec3(f);

		ap[1] = Sqr(1 - f) * T;

		// ap[2] = ap[1] * T * f;
		for (int p = 2; p < pMax; ++p) {
			ap[p] = ap[p - 1] * T * f;
		}

		ap[pMax] = ap[pMax - 1] * f * T / (Vec3(1.0) - T * f);

		return ap;
	}

	inline std::array<double, pMax + 1> betam_to_v(double bm) {
		std::array<double, pMax + 1> vs;
		vs[0] = Sqr(0.726 * bm + 0.812 * Sqr(bm) + 3.7 * Pow<20>(bm));
		vs[1] = 0.25 * vs[0];
		vs[2] = 4.0 * vs[0];
		for (int p = 3; p <= pMax; ++p)
			vs[p] = vs[2];

		return vs;
	}

	/*
	初期角度(phiO)を0と基準にしたときの、ピーク角度を返す。
	*/
	inline double Phi(int p, double gammaO, double gammaT) {
		return 2.0 * p * gammaT - 2.0 * gammaO + p * glm::pi<double>();
	}

	inline double Logistic(double x, double s) {
		x = std::abs(x);
		double exp_minus_x_over_s = std::exp(-x / s);
		return exp_minus_x_over_s / (s * Sqr(1.0 + exp_minus_x_over_s));
	}
	inline double LogisticCDF(double x, double s) {
		return 1.0 / (1.0 + std::exp(-x / s));
	}

	inline double TrimmedLogistic(double x, double s, double a, double b) {
		return Logistic(x, s) / (LogisticCDF(b, s) - LogisticCDF(a, s));
	}
	inline double Np(double phi, int p, double s, double gammaO, double gammaT) {
		/*
		 ここで phi は phiOからphiIへの相対角度
		*/
		double dphi = phi - Phi(p, gammaO, gammaT);

		while (dphi > glm::pi<double>()) dphi -= 2.0 * glm::pi<double>();
		while (dphi < -glm::pi<double>()) dphi += 2.0 * glm::pi<double>();

		return TrimmedLogistic(dphi, s, -glm::pi<double>(), glm::pi<double>());
	}

	inline double betan_to_s(double beta_n) {
		static const double SqrtPiOver8 = 0.626657069;
		return SqrtPiOver8 * (0.265 * beta_n + 1.194 * Sqr(beta_n) + 5.372 * Pow<22>(beta_n));
	}
	inline double AbsCosThetaForFur(const Vec3 &w) {
		return glm::sqrt(w.z * w.z + w.y * w.y);
	}

	inline Mat3 to_fur_bsdf_basis_transform(const Vec3 &tangent) {
		Vec3 yaxis;
		Vec3 xaxis = tangent;
		Vec3 zaxis;

		if (0.999 < glm::abs(xaxis.z)) {
			yaxis = glm::normalize(glm::cross(xaxis, Vec3(0.0, 1.0, 0.0)));
		}
		else {
			yaxis = glm::normalize(glm::cross(xaxis, Vec3(0.0, 0.0, 1.0)));
		}
		zaxis = glm::cross(xaxis, yaxis);

		Mat3 from_bsdf(xaxis, yaxis, zaxis);
		return glm::transpose(from_bsdf);
	}

	struct FurBSDFParams {
		double eta = 1.55;
		double beta_n = 0.9;
		double beta_m = 0.9;
		double alpha = 0.000;
		Vec3 sigma_a;
		double h = 0.0;
	};
	/*
	woがカメラ方向
	wiがライト方向
	*/
	inline glm::dvec3 fur_bsdf(glm::dvec3 wi, glm::dvec3 wo, const FurBSDFParams &params) {
		double h = params.h;
		double eta = params.eta;
		double beta_n = params.beta_n;
		double beta_m = params.beta_m;
		double alpha = params.alpha;
		Vec3 sigma_a = params.sigma_a;

		double sinThetaO = wo.x;
		double cosThetaO = SafeSqrt(1.0 - Sqr(sinThetaO));
		double phiO = std::atan2(wo.z, wo.y);

		double sinThetaI = wi.x;
		double cosThetaI = SafeSqrt(1.0 - Sqr(sinThetaI));
		double phiI = std::atan2(wi.z, wi.y);

		double s = betan_to_s(beta_n);
		
		double sinThetaT = sinThetaO / eta;
		double cosThetaT = SafeSqrt(1.0 - Sqr(sinThetaT));

		double etap = glm::sqrt(eta * eta - Sqr(sinThetaO)) / cosThetaO;
		double sinGammaT = h / etap;
		double cosGammaT = SafeSqrt(1 - Sqr(sinGammaT));
		double gammaT = SafeASin(sinGammaT);

		double l = 2.0 * cosGammaT / cosThetaT;

		Vec3 T = glm::exp(-sigma_a * l);

		
		std::array<double, 3> sin2kAlpha;
		std::array<double, 3> cos2kAlpha;
		sin2kAlpha[0] = std::sin(alpha);
		cos2kAlpha[0] = SafeSqrt(1 - Sqr(sin2kAlpha[0]));
		for (int i = 1; i < 3; ++i) {
			sin2kAlpha[i] = 2 * cos2kAlpha[i - 1] * sin2kAlpha[i - 1];
			cos2kAlpha[i] = Sqr(cos2kAlpha[i - 1]) - Sqr(sin2kAlpha[i - 1]);
		}

		double gammaO = SafeASin(h);

		double phi = phiI - phiO;
		std::array<Vec3, pMax + 1> ap = Ap(cosThetaO, eta, h, T);
		std::array<double, pMax + 1> v = betam_to_v(beta_m);
		Vec3 fsum(0.);
		for (int p = 0; p < 3; ++p) {
			double sinThetaIp, cosThetaIp;
			if (p == 0) {
				sinThetaIp = sinThetaI * cos2kAlpha[1] + cosThetaI * sin2kAlpha[1];
				cosThetaIp = cosThetaI * cos2kAlpha[1] - sinThetaI * sin2kAlpha[1];
			}
			else if (p == 1) {
				sinThetaIp = sinThetaI * cos2kAlpha[0] - cosThetaI * sin2kAlpha[0];
				cosThetaIp = cosThetaI * cos2kAlpha[0] + sinThetaI * sin2kAlpha[0];
			}
			else if (p == 2) {
				sinThetaIp = sinThetaI * cos2kAlpha[2] - cosThetaI * sin2kAlpha[2];
				cosThetaIp = cosThetaI * cos2kAlpha[2] + sinThetaI * sin2kAlpha[2];
			}
			else {
				sinThetaIp = sinThetaI;
				cosThetaIp = cosThetaI;
			}
			cosThetaIp = std::abs(cosThetaIp);

			fsum += Mp(sinThetaIp, cosThetaIp, sinThetaO, cosThetaO, v[p]) * ap[p] * Np(phi, p, s, gammaO, gammaT);
		}

		fsum += Mp(cosThetaI, cosThetaO, sinThetaI, sinThetaO, v[pMax]) * ap[pMax] / (2.0 * glm::pi<double>());
		if (AbsCosThetaForFur(wi) > 0) {
			fsum /= AbsCosThetaForFur(wi);
		}
		return fsum;
	}

	/*
	Mpの重点サンプリング
	eps1, eps2 = 0~1 乱数
	θを返す
	*/
	inline double sampleMp(double eps1, double eps2, double v, double sinThetaO) {
		// double u = v * glm::log(glm::exp(1.0 / v) - 2.0 * eps1 * glm::sinh(1.0 / v));
		// Numerically stable sampling of the von Mises Fisher distribution on S2 (and other tricks) ver
		// avoids overflow ver.
		double u = 1.0 + v * glm::log(eps1 + (1.0 - eps1) * glm::exp(-2.0 / v));

		double theta_cone = -glm::asin(sinThetaO);
		double theta_tap = glm::pi<double>() * 0.5 - theta_cone;
		return glm::asin(u * glm::cos(theta_tap) + glm::sqrt(1.0 - u * u) * glm::cos(glm::two_pi<double>() * eps2) * glm::sin(theta_tap));
	}

	/*
	TrimmedLogisticの重点サンプリング
	eps1 = 0~1 乱数
	x を返す
	*/
	inline double sampleTrimmedLogistic(double eps1, double s, double a, double b) {
		double T = LogisticCDF(b, s) - LogisticCDF(a, s);
		return -s * glm::log(
			1.0 / (eps1 * T + 1.0 / (1.0 + glm::exp(-a / s))) - 1.0
		);
	}

	inline double constrainRadian(double x) {
		x = fmod(x + glm::pi<double>(), glm::two_pi<double>());
		if (x < 0.0) {
			x += glm::two_pi<double>();
		}
		return x - glm::pi<double>();
	}

	/*
	  Npの重点サンプリング
	  phiI を返す
	*/
	inline double sampleNp(double eps1, double phiO, int p, double s, double gammaO, double gammaT) {
		double dphi = Phi(p, gammaO, gammaT) + sampleTrimmedLogistic(eps1, s, -glm::pi<double>(), glm::pi<double>());
		double phiI = phiO + dphi;

		phiI = constrainRadian(phiI);

		return phiI;
	}

	inline double luminance(const Vec3 &color) {
		// srgb to y
		return color.r * 0.212639 + color.g * 0.715168 + color.b * 0.072192;
	}

	inline std::array<double, pMax + 1> toApTap(const std::array<Vec3, pMax + 1> &ap) {
		std::array<double, pMax + 1> r;
		for (int i = 0; i < r.size(); ++i) {
			r[i] = luminance(ap[i]);
		}
		return r;
	}
	inline std::array<double, pMax + 1> toApPDF(const std::array<double, pMax + 1> &ap_tap) {
		double sum = 0.0;
		for (int i = 0; i < ap_tap.size(); ++i) {
			sum += ap_tap[i];
		}
		std::array<double, pMax + 1> r = ap_tap;
		for (int i = 0; i < r.size(); ++i) {
			r[i] /= sum;
		}
		return r;
	}
	inline std::array<double, pMax + 1> toApCDF(const std::array<double, pMax + 1> &apPDF) {
		std::array<double, pMax + 1> r;
		double sum = 0.0;
		for (int i = 0; i < apPDF.size(); ++i) {
			sum += apPDF[i];
			r[i] = sum;
		}
		return r;
	}
	inline int sampleAp(double eps1, const std::array<double, pMax + 1> &apCDF) {
		for (int i = 0; i < apCDF.size(); ++i) {
			if (eps1 < apCDF[i]) {
				return i;
			}
		}
		return apCDF.size() - 1;
	}

	inline Vec3 sampleFur(std::array<double, 4> eps_x4, const Vec3 &wo, const FurBSDFParams &params, double *p_omega) {
		double h = params.h;
		double eta = params.eta;
		double beta_n = params.beta_n;
		double beta_m = params.beta_m;
		double alpha = params.alpha;
		Vec3 sigma_a = params.sigma_a;

		double sinThetaO = wo.x;
		double cosThetaO = SafeSqrt(1.0 - Sqr(sinThetaO));
		double phiO = std::atan2(wo.z, wo.y);

		double s = betan_to_s(beta_n);

		double sinThetaT = sinThetaO / eta;
		double cosThetaT = SafeSqrt(1.0 - Sqr(sinThetaT));

		double etap = glm::sqrt(eta * eta - Sqr(sinThetaO)) / cosThetaO;
		double sinGammaT = h / etap;
		double cosGammaT = SafeSqrt(1 - Sqr(sinGammaT));
		double gammaT = SafeASin(sinGammaT);

		double l = 2.0 * cosGammaT / cosThetaT;

		Vec3 T = glm::exp(-sigma_a * l);

		std::array<double, 3> sin2kAlpha;
		std::array<double, 3> cos2kAlpha;
		sin2kAlpha[0] = std::sin(alpha);
		cos2kAlpha[0] = SafeSqrt(1 - Sqr(sin2kAlpha[0]));
		for (int i = 1; i < 3; ++i) {
			sin2kAlpha[i] = 2 * cos2kAlpha[i - 1] * sin2kAlpha[i - 1];
			cos2kAlpha[i] = Sqr(cos2kAlpha[i - 1]) - Sqr(sin2kAlpha[i - 1]);
		}

		double gammaO = SafeASin(h);

		std::array<Vec3, pMax + 1> ap = Ap(cosThetaO, eta, h, T);
		std::array<double, pMax + 1> ap_tap = toApTap(ap);
		std::array<double, pMax + 1> apPDF = toApPDF(ap_tap);
		std::array<double, pMax + 1> apCDF = toApCDF(apPDF);

		std::array<double, pMax + 1> v = betam_to_v(beta_m);

		// サンプリング
		int sample_p = sampleAp(eps_x4[0], apCDF);
		double thetaI = sampleMp(eps_x4[1], eps_x4[2], v[sample_p], sinThetaO);
		double phiI;
		if (sample_p < pMax) {
			phiI = sampleNp(eps_x4[3], phiO, sample_p, s, gammaO, gammaT);
		} else {
			phiI = glm::mix(-glm::pi<double>(), glm::pi<double>(), eps_x4[3]);
		}

		// Alphaへの対応がちょっと自信がない
		//double thetaI_Alpha;
		//switch (sample_p) {
		//case 0:
		//	thetaI_Alpha = thetaI + 2.0 * alpha;
		//	break;
		//case 1:
		//	thetaI_Alpha = thetaI - alpha;
		//	break;
		//case 2:
		//	thetaI_Alpha = thetaI - 4.0 * alpha;
		//	break;
		//default:
		//	thetaI_Alpha = thetaI;
		//	break;
		//}

		double sinThetaI = glm::sin(thetaI);
		double cosThetaI = glm::cos(thetaI);

		double phi = phiI - phiO;

		double p_omega_sum = 0.0;
		for (int p = 0; p < 3; ++p) {
			double sinThetaIp, cosThetaIp;
			if (p == 0) {
				sinThetaIp = sinThetaI * cos2kAlpha[1] + cosThetaI * sin2kAlpha[1];
				cosThetaIp = cosThetaI * cos2kAlpha[1] - sinThetaI * sin2kAlpha[1];
			}
			else if (p == 1) {
				sinThetaIp = sinThetaI * cos2kAlpha[0] - cosThetaI * sin2kAlpha[0];
				cosThetaIp = cosThetaI * cos2kAlpha[0] + sinThetaI * sin2kAlpha[0];
			}
			else if (p == 2) {
				sinThetaIp = sinThetaI * cos2kAlpha[2] - cosThetaI * sin2kAlpha[2];
				cosThetaIp = cosThetaI * cos2kAlpha[2] + sinThetaI * sin2kAlpha[2];
			}
			else {
				sinThetaIp = sinThetaI;
				cosThetaIp = cosThetaI;
			}
			cosThetaIp = std::abs(cosThetaIp);

			p_omega_sum += Mp(sinThetaIp, cosThetaIp, sinThetaO, cosThetaO, v[p]) * apPDF[p] * Np(phi, p, s, gammaO, gammaT);
		}

		p_omega_sum += Mp(cosThetaI, cosThetaO, sinThetaI, sinThetaO, v[pMax]) * apPDF[pMax] / (2.0 * glm::pi<double>());
	
		*p_omega = p_omega_sum;

		Vec3 wi = Vec3(sinThetaI, cosThetaI * std::cos(phiI), cosThetaI * std::sin(phiI));
		return wi;
	}
	inline double pdfFur(const Vec3 &wi, const Vec3 &wo, const FurBSDFParams &params) {
		double h = params.h;
		double eta = params.eta;
		double beta_n = params.beta_n;
		double beta_m = params.beta_m;
		double alpha = params.alpha;
		Vec3 sigma_a = params.sigma_a;

		double sinThetaO = wo.x;
		double cosThetaO = SafeSqrt(1.0 - Sqr(sinThetaO));
		double phiO = std::atan2(wo.z, wo.y);

		double s = betan_to_s(beta_n);

		double sinThetaT = sinThetaO / eta;
		double cosThetaT = SafeSqrt(1.0 - Sqr(sinThetaT));

		double etap = glm::sqrt(eta * eta - Sqr(sinThetaO)) / cosThetaO;
		double sinGammaT = h / etap;
		double cosGammaT = SafeSqrt(1 - Sqr(sinGammaT));
		double gammaT = SafeASin(sinGammaT);

		double l = 2.0 * cosGammaT / cosThetaT;

		Vec3 T = glm::exp(-sigma_a * l);

		std::array<double, 3> sin2kAlpha;
		std::array<double, 3> cos2kAlpha;
		sin2kAlpha[0] = std::sin(alpha);
		cos2kAlpha[0] = SafeSqrt(1 - Sqr(sin2kAlpha[0]));
		for (int i = 1; i < 3; ++i) {
			sin2kAlpha[i] = 2 * cos2kAlpha[i - 1] * sin2kAlpha[i - 1];
			cos2kAlpha[i] = Sqr(cos2kAlpha[i - 1]) - Sqr(sin2kAlpha[i - 1]);
		}

		double gammaO = SafeASin(h);

		std::array<Vec3, pMax + 1> ap = Ap(cosThetaO, eta, h, T);
		std::array<double, pMax + 1> ap_tap = toApTap(ap);
		std::array<double, pMax + 1> apPDF = toApPDF(ap_tap);
		std::array<double, pMax + 1> apCDF = toApCDF(apPDF);

		std::array<double, pMax + 1> v = betam_to_v(beta_m);

		double sinThetaI = wi.x;
		double cosThetaI = SafeSqrt(1.0 - Sqr(sinThetaI));
		double phiI = std::atan2(wi.z, wi.y);

		double phi = phiI - phiO;

		double p_omega_sum = 0.0;
		for (int p = 0; p < 3; ++p) {
			double sinThetaIp, cosThetaIp;
			if (p == 0) {
				sinThetaIp = sinThetaI * cos2kAlpha[1] + cosThetaI * sin2kAlpha[1];
				cosThetaIp = cosThetaI * cos2kAlpha[1] - sinThetaI * sin2kAlpha[1];
			}
			else if (p == 1) {
				sinThetaIp = sinThetaI * cos2kAlpha[0] - cosThetaI * sin2kAlpha[0];
				cosThetaIp = cosThetaI * cos2kAlpha[0] + sinThetaI * sin2kAlpha[0];
			}
			else if (p == 2) {
				sinThetaIp = sinThetaI * cos2kAlpha[2] - cosThetaI * sin2kAlpha[2];
				cosThetaIp = cosThetaI * cos2kAlpha[2] + sinThetaI * sin2kAlpha[2];
			}
			else {
				sinThetaIp = sinThetaI;
				cosThetaIp = cosThetaI;
			}
			cosThetaIp = std::abs(cosThetaIp);

			p_omega_sum += Mp(sinThetaIp, cosThetaIp, sinThetaO, cosThetaO, v[p]) * apPDF[p] * Np(phi, p, s, gammaO, gammaT);
		}
		p_omega_sum += Mp(cosThetaI, cosThetaO, sinThetaI, sinThetaO, v[pMax]) * apPDF[pMax] / (2.0 * glm::pi<double>());

		return p_omega_sum;
	}
}