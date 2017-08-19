#pragma once

#include <tuple>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "geometry.hpp"

namespace rt {
	class OnlineCovarianceMatrix2x2 {
	public:
		void addSample(Vec2 x) {
			_00.addSample(x[0], x[0]);
			_11.addSample(x[1], x[1]);
			_01.addSample(x[0], x[1]);
		}
		Eigen::Matrix2d sampleCovarianceMatrix() const {
			Eigen::Matrix2d cov;
			cov(0, 0) = _00.sampleCovariance();
			cov(1, 1) = _11.sampleCovariance();
			cov(0, 1) = cov(1, 0) = _01.sampleCovariance();
			return cov;
		}
	private:
		OnlineCovariance _00;
		OnlineCovariance _11;
		OnlineCovariance _01;
	};
	class OnlineCovarianceMatrix3x3 {
	public:
		void addSample(Vec3 x) {
			_00.addSample(x[0], x[0]);
			_11.addSample(x[1], x[1]);
			_22.addSample(x[2], x[2]);
			_01.addSample(x[0], x[1]);
			_02.addSample(x[0], x[2]);
			_12.addSample(x[1], x[2]);
		}
		Eigen::Matrix3d sampleCovarianceMatrix() const {
			Eigen::Matrix3d cov;
			cov(0, 0) = _00.sampleCovariance();
			cov(1, 1) = _11.sampleCovariance();
			cov(2, 2) = _22.sampleCovariance();
			cov(0, 1) = cov(1, 0) = _01.sampleCovariance();
			cov(0, 2) = cov(2, 0) = _02.sampleCovariance();
			cov(1, 2) = cov(2, 1) = _12.sampleCovariance();
			return cov;
		}
	private:
		OnlineCovariance _00;
		OnlineCovariance _11;
		OnlineCovariance _22;
		OnlineCovariance _01;
		OnlineCovariance _02;
		OnlineCovariance _12;
	};

	/*
	主成分分析 2x2 mat
	*/
	inline std::tuple<Vec2, Vec2> PCA(const Eigen::Matrix2d covarianceMatrix) {
		Eigen::EigenSolver<Eigen::Matrix2d> es(covarianceMatrix);
		auto eigenvectors = es.eigenvectors();
		Vec2 xaxis(
			eigenvectors(0, 0).real(),
			eigenvectors(1, 0).real()
		);
		Vec2 yaxis(
			eigenvectors(0, 1).real(),
			eigenvectors(1, 1).real()
		);
		return{ xaxis, yaxis };
	}

	/*
	主成分分析 3x3 mat
	*/
	inline std::tuple<Vec3, Vec3, Vec3> PCA(const Eigen::Matrix3d covarianceMatrix) {
		Eigen::EigenSolver<Eigen::Matrix3d> es(covarianceMatrix);
		auto eigenvectors = es.eigenvectors();
		Vec3 xaxis(
			eigenvectors(0, 0).real(),
			eigenvectors(1, 0).real(),
			eigenvectors(2, 0).real()
		);
		Vec3 yaxis(
			eigenvectors(0, 1).real(),
			eigenvectors(1, 1).real(),
			eigenvectors(2, 1).real()
		);
		Vec3 zaxis(
			eigenvectors(0, 2).real(),
			eigenvectors(1, 2).real(),
			eigenvectors(2, 2).real()
		);

		return{ xaxis , yaxis, zaxis };
	}
}