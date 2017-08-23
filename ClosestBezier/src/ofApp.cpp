#include "ofApp.h"

static rt::Xor random;
static auto r = []() {
	double w = ofGetWidth();
	double h = ofGetHeight();
	return rt::Vec3(random.uniform(0, w), random.uniform(0, h), 0.0);
};

/*
 http://muses.muses.tottori-u.ac.jp/faculty/ogata/lecture/numerical-analysis.pdf
 ÇÊÇË
 ax^3 + bx^2 + cx + d == 0
 ÇÃç≈è¨0å¬ ç≈ëÂÇRå¬Å@é¿êîâÇãÅÇﬂÇÈÅBâÇÃå¬êîÇï‘Ç∑
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
			} else {
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
		} else {
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
		double s = std::fma(-n, 0.5,  sqrtD);
		double t = std::fma(-n, 0.5, -sqrtD);
		double lhs = s < 0.0 ? -std::pow(-s, 1.0 / 3.0) : std::pow(s, 1.0 / 3.0);
		double rhs = t < 0.0 ? -std::pow(-t, 1.0 / 3.0) : std::pow(t, 1.0 / 3.0);
		z[0] = lhs + rhs;
		N = 1;
	} else {
		// D < 0
		// double arg = glm::sqrt((n * n / 4.0) / (- m * m * m / 27.0));
		double arg = std::sqrt(- nn_over_4 * 27.0 / mmm);
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

//--------------------------------------------------------------
void ofApp::setup(){
	_bezier[0] = r();
	_bezier[1] = r();
	_bezier[2] = r();

	double max_error1 = 0;
	double max_error2 = 0;
	int max_index = -1;
	for (int i = 0; i < 100000; ++i) {
		double a = random.uniform(-10, 10);
		double b = random.uniform(-10, 10);
		double c = random.uniform(-10, 10);
		double d = random.uniform(-10, 10);

		double r1[3];
		double r2[3];
		int n = cardano_solve(a, b, c, d, r1);
		cardano_solve_and_newton(a, b, c, d, r2);

		for (int j = 0; j < n; ++j) {
			double x1 = r1[j];
			double x2 = r2[j];
			double zero1 = cubic_evaluate(x1, a, b, c, d);
			double zero2 = cubic_evaluate(x2, a, b, c, d);

			//if (glm::abs(zero1) > 1e-9) {
			//	printf("%.20f -> %.20f (%.20f)\n", glm::abs(zero1), glm::abs(zero2), glm::abs(zero2) - glm::abs(zero1));
			//}

			max_error1 = std::max(max_error1, glm::abs(zero1));
			if (max_error2 < glm::abs(zero2)) {
				max_error2 = glm::abs(zero2);
				max_index = i;
			}
			// max_error2 = std::max(max_error2, glm::abs(zero2));
			if (glm::abs(zero1) > 0.0001) {
				printf("error : %.10f\n", glm::abs(zero1));
			}
			if (glm::abs(zero2) > 0.00001) {
				printf("error : %.10f\n", glm::abs(zero2));
			}
		}
	}
	printf("max error 1 : %.10f\n", max_error1);
	printf("max error 2 : %.10f, [%d]\n", max_error2, max_index);

}

//--------------------------------------------------------------
void ofApp::update(){

}



//--------------------------------------------------------------
void ofApp::draw(){
	ofClear(0);

	auto drawBezier = [=](rt::BezierQuadratic3D bz) {
		{
			ofSetColor(ofColor::white);
			ofDrawSphere(bz[0].x, bz[0].y, bz[0].z, 3);

			ofSetColor(ofColor::red);
			ofDrawSphere(bz[2].x, bz[2].y, bz[2].z, 3);

		}
		ofSetColor(ofColor::orange);

		ofNoFill();

		ofPolyline line;
		int N = 40;
		for (int i = 0; i < N; ++i) {
			float t = (float)i / (N - 1);
			auto p = bz.evaluate(t);
			line.addVertex(p.x, p.y, p.z);
		}

		line.draw();

		ofFill();
	};

	drawBezier(_bezier);

	/*
	auto p0 = rt::Vec2(_bezier[0]);
	auto p1 = rt::Vec2(_bezier[1]);
	auto p2 = rt::Vec2(_bezier[2]);
	auto A = p1 - p0;
	auto B = p2 - p1 - A;

	auto M = rt::Vec2(ofGetMouseX(), ofGetMouseY());

	auto M_tap = p0 - M;
	{
		auto a = glm::dot(B, B);
		auto b = 3.0 * glm::dot(A, B);
		auto c = 2.0 * glm::dot(A, A) + glm::dot(M_tap, B);
		auto d = glm::dot(M_tap, A);

		double t[3];
		int N = cardano_solve(a, b, c, d, t);
		// int count = cardano_solve_and_newton(a, b, c, d, r);

		//ofSetColor(ofColor::green);
		//for (int i = 0; i < N; ++i) {
		//	auto p3 = _bezier.evaluate(t[i]);
		//	ofDrawSphere(p3.x, p3.y, 5.0f);
		//}

		if (0 < N) {
			double closest_t = glm::clamp(t[0], 0.0, 1.0);
			double closestDistanceSq = glm::distance2(rt::Vec2(_bezier.evaluate(closest_t)), M);
			for (int i = 1; i < N; ++i) {
				double this_t = glm::clamp(t[i], 0.0, 1.0);
				double distanceSq = glm::distance2(rt::Vec2(_bezier.evaluate(this_t)), M);
				if (distanceSq < closestDistanceSq) {
					closest_t = this_t;
					closestDistanceSq = distanceSq;
				}
			}

			auto closest = rt::Vec2(_bezier.evaluate(closest_t));

			ofSetColor(ofColor::green);
			ofDrawSphere(closest.x, closest.y, 5.0f);
			ofLine(M.x, M.y, closest.x, closest.y);
		}
	}

	ofSetColor(ofColor::paleGreen);
	ofDrawCircle(M.x, M.y, 5.0);
	*/

	// http://blog.gludion.com/2009/08/distance-to-quadratic-bezier-curve.html
	auto p0 = rt::Vec2(_bezier[0]);
	auto p1 = rt::Vec2(_bezier[1]);
	auto p2 = rt::Vec2(_bezier[2]);
	auto A = p1 - p0;
	auto B = p2 - p1 - A;

	double w = ofGetWidth();
	double h = ofGetHeight();
	double elapsed = ofGetElapsedTimef() * 0.03;

	for (int i = 0; i < 10000; ++i) {
		auto M = rt::Vec2(ofNoise(elapsed, i * 0.45) * w, ofNoise(i * 0.45, 0.4, elapsed) * h);

		auto M_tap = p0 - M;
		{
			auto a = glm::dot(B, B);
			auto b = 3.0 * glm::dot(A, B);
			auto c = 2.0 * glm::dot(A, A) + glm::dot(M_tap, B);
			auto d = glm::dot(M_tap, A);

			double t[3];
			int N = cardano_solve(a, b, c, d, t);
			// int count = cardano_solve_and_newton(a, b, c, d, r);

			//ofSetColor(ofColor::green);
			//for (int i = 0; i < N; ++i) {
			//	auto p3 = _bezier.evaluate(t[i]);
			//	ofDrawSphere(p3.x, p3.y, 5.0f);
			//}

			if (0 < N) {
				double closest_t = glm::clamp(t[0], 0.0, 1.0);
				double closestDistanceSq = glm::distance2(rt::Vec2(_bezier.evaluate(closest_t)), M);
				for (int i = 1; i < N; ++i) {
					double this_t = glm::clamp(t[i], 0.0, 1.0);
					double distanceSq = glm::distance2(rt::Vec2(_bezier.evaluate(this_t)), M);
					if (distanceSq < closestDistanceSq) {
						closest_t = this_t;
						closestDistanceSq = distanceSq;
					}
				}

				auto closest = rt::Vec2(_bezier.evaluate(closest_t));
				double d = glm::distance(closest, M);
				ofColor C = ofColor::fromHsb(255 * glm::clamp(1.0 - d / 200, 0.1, 1.0), 255, 255);
				ofSetColor(C);
				// ofSetColor(ofColor::green * glm::clamp(1.0 - d / 200, 0.1, 1.0));
				// ofDrawSphere(closest.x, closest.y, 5.0f);
				// ofLine(M.x, M.y, closest.x, closest.y);

				ofDrawCircle(M.x, M.y, 2.0);
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == ' ') {
		_bezier[0] = r();
		_bezier[1] = r();
		_bezier[2] = r();
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
