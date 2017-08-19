#pragma once

#include "ofMain.h"
#include "ofxImGui.h"

#include "scene.hpp"
#include "conelbox.hpp"
#include "stopwatch.hpp"
#include "bezier.hpp"
#include "stats.hpp"
#include "aabb.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

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

		return { xaxis , yaxis, zaxis };
	}

	struct OBB {
		// 中心
		Vec3 ac;

		// 軸
		std::array<Vec3, 3> a;

		// サイズの幅の半分
		Vec3 h;

		void expand(double dxyz) {
			h += Vec3(dxyz);
		}

		double surface_area() const {
			Vec3 s = h * 2.0;
			return (s.x * s.z + s.x * s.y + s.z * s.y) * 2.0;
		}
	};

	inline std::experimental::optional<double> intersect_obb(const Ray &ray, const OBB &obb) {
		auto P = obb.ac - ray.o;
		double e0 = glm::dot(P, obb.a[0]);
		double one_over_cosTheta0 = 1.0 / glm::dot(ray.d, obb.a[0]);
		double tmin0 = (e0 - obb.h[0]) * one_over_cosTheta0;
		double tmax0 = (e0 + obb.h[0]) * one_over_cosTheta0;
		if (tmax0 < tmin0) { std::swap(tmax0, tmin0); }

		double e1 = glm::dot(P, obb.a[1]);
		double one_over_cosTheta1 = 1.0 / glm::dot(ray.d, obb.a[1]);
		double tmin1 = (e1 - obb.h[1]) * one_over_cosTheta1;
		double tmax1 = (e1 + obb.h[1]) * one_over_cosTheta1;
		if (tmax1 < tmin1) { std::swap(tmax1, tmin1); }

		double e2 = glm::dot(P, obb.a[2]);
		double one_over_cosTheta2 = 1.0 / glm::dot(ray.d, obb.a[2]);
		double tmin2 = (e2 - obb.h[2]) * one_over_cosTheta2;
		double tmax2 = (e2 + obb.h[2]) * one_over_cosTheta2;
		if (tmax2 < tmin2) { std::swap(tmax2, tmin2); }

		double tmin = glm::max(glm::max(tmin0, tmin1), tmin2);
		double tmax = glm::min(glm::min(tmax0, tmax1), tmax2);

		// tmin <= tmax は直線が交点を持っている
		// tmin < 0.0
		//    :: tmaxが交点
		// 0.0 < tmin
		//    :: tminが交点
		// tmax < 0.0
		//    :: 当たっていない

		bool intersect = tmin <= tmax && 0.0 < tmax;
		if (intersect) {
			return tmin < 0.0 ? tmax : tmin;
		}
		return std::experimental::optional<double>();
	}

	class OnlineOBB {
	public:
		OnlineOBB(const std::array<Vec3, 3> &axis):_axis(axis) {

		}

		void addSample(const Vec3 &p) {
			for (int i = 0; i < 3; ++i) {
				double projected = glm::dot(_axis[i], p);
				_min[i] = glm::min(_min[i], projected);
				_max[i] = glm::max(_max[i], projected);
			}
		}
		OBB obb() const {
			Vec3 ac_local = (_max + _min) * 0.5;

			OBB o;
			o.ac = _axis[0] * ac_local[0] + _axis[1] * ac_local[1] + _axis[2] * ac_local[2];
			o.a = _axis;
			o.h = (_max - _min) * 0.5;
			return o;
		}

		std::array<Vec3, 3> _axis;
		Vec3 _min = Vec3(std::numeric_limits<double>::max());
		Vec3 _max = Vec3(-std::numeric_limits<double>::max());
	};
}

struct BeizerFur {
	double radius = 0.0;
	rt::BezierQuadratic3D bezier;
};
struct BezierLeaf {
	std::vector<BeizerFur> beizers;
};

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofEasyCam _camera;
	ofxImGui::Gui _imgui;
	ofImage _image;

	std::shared_ptr<rt::Scene> _scene;
	rt::BezierQuadratic3D _bezierQuadratic;

	
	int _index = 0;
	std::vector<BezierLeaf> _leafs;

	float _ray_y = 0.0f;
	float _ray_z = 0.0f;
};
