#include "ofApp.h"

static rt::Xor random;
static auto r = []() {
	double w = ofGetWidth();
	double h = ofGetHeight();
	return rt::Vec3(random.uniform(0, w), random.uniform(0, h), 0.0);
};


//--------------------------------------------------------------
void ofApp::setup(){
	_bezier[0] = r();
	_bezier[1] = r();
	_bezier[2] = r();
	using namespace rt;

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
	using namespace rt;

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
