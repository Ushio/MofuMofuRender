#include "ofApp.h"

#include "scene.hpp"
#include "peseudo_random.hpp"
using namespace rt;

inline ofVec3f toOf(rt::Vec3 v) {
	return ofVec3f(v.x, v.y, v.z);
}

//--------------------------------------------------------------
void ofApp::setup() {
	_imgui.setup();

	_camera.setNearClip(0.1f);
	_camera.setFarClip(100.0f);
	_camera.setDistance(5.0f);
}

//--------------------------------------------------------------
void ofApp::update(){

}

inline double g(double x, double sigma2, double u) {
	double d = x - u;
	return glm::exp(-d*d / (2.0 * sigma2));
}
inline double sg(Vec3 x, double v, Vec3 p) {
	return g(glm::distance(x, p), v, 0.0);
}

inline double sg_2(Vec3 x, double v, Vec3 p) {
	//return glm::exp((glm::dot(x, p) - 1.0) / v);
	return glm::exp(glm::dot(x, p) / v) * glm::exp(-1.0 / v);
}

inline double sg_normalized(Vec3 x, double v, Vec3 p) {
	double C = 1.0 / (
		4.0 * glm::pi<double>() * v * sinh(1.0/v)
	);
	return C * glm::exp((glm::dot(x, p)) / v);
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofClear(0);

	_camera.begin();

	ofPushMatrix();
	ofRotateZ(90.0f);
	ofSetColor(255);
	ofDrawGridPlane(1.0f);
	ofPopMatrix();

	ofPushMatrix();
	ofSetLineWidth(3);
	ofDrawAxis(1);
	ofSetLineWidth(1);
	ofPopMatrix();

	static ofMesh mesh;
	mesh.clear();
	mesh.setMode(OF_PRIMITIVE_POINTS);

	//Vec3 P = glm::normalize(Vec3(1, 1, 0));
	//double v = _v;
	//Xor random;
	//for (int i = 0; i < 5000; ++i) {
	//	Vec3 x = uniform_on_unit_sphere(&random);
	//	double value = _check ? sg(x, v, P) : sg_2(x, v, P);

	//	double csch = 1.0 / glm::sinh(1.0 / v);
	//	double same = csch / (2.0 * v) * glm::exp(glm::dot(P, x) / v);

	//	// double value = _check ? same : sg_normalized(x, v, P);
	//	// value = same;
	//	//printf("df %.5f\n", glm::abs(same - value));
	//	//if (_check) {
	//	//	value = same;
	//	//}

	//	auto plot = x * value;
	//	mesh.addVertex(toOf(plot));
	//}
	//ofSetColor(255);
	//mesh.draw();

	{
		Xor random;
		double v = _v;
		for (int i = 0; i < 10000; ++i) {
			// 純粋なフォンミーゼス分布のサンプリング
			// P(0, 0, 1)
			double eps1 = random.uniform();
			double eps2 = random.uniform();

			double W = v * glm::log(glm::exp(-1.0 / v) + 2.0 * eps1 * glm::sinh(1.0 / v));
			Vec2 V = Vec2(glm::cos(eps2 * glm::two_pi<double>()), glm::sin(eps2 * glm::two_pi<double>()));
			
			double r = glm::sqrt(1.0 - W * W);
			Vec3 omega(
				r * V.x,
				r * V.y,
				W
			);
			omega = glm::rotateX(omega, random.uniform(0.0, glm::two_pi<double>()));
			mesh.addVertex(toOf(omega));
		}
		ofSetColor(255);
		mesh.draw();
	}

	//{
	//	rt::Xor random;
	//	Vec3 P = uniform_on_unit_sphere(&random);

	//	Vec3 sum;
	//	int count = 100000;
	//	for (int i = 0; i < count; ++i) {

	//		double p_omega = 1.0 / (4.0 * glm::pi<double>());
	//		Vec3 sigma_a;
	//		Vec3 x = uniform_on_unit_sphere(&random);

	//		double csch = 1.0 / glm::sinh(1.0 / v);
	//		double same = csch / (2.0 * v) * glm::exp(glm::dot(P, x) / v);

	//		sum += sg_normalized(x, v, P)  / p_omega;
	//		// same
	//	}

	//	double avg = sum.y / count;
	//	printf("%.3f\n", avg);
	//}

	{
		double r = cos(_theta);
		Vec3 P(
			sin(_theta),
			r * cos(_phi),
			r * sin(_phi)
		);

		ofSetColor(ofColor::orange);
		ofDrawLine(ofVec3f(), toOf(P));
	}



	_camera.end();


	_imgui.begin();

	ImGui::PushStyleColor(ImGuiCol_WindowBg, ofVec4f(0.2f, 0.2f, 0.5f, 0.5f));
	ImGui::SetNextWindowPos(ofVec2f(500, 30), ImGuiSetCond_Once);
	ImGui::SetNextWindowSize(ofVec2f(500, 600), ImGuiSetCond_Once);

	ImGui::Begin("Config Panel");
	ImGui::Text("fps: %.2f", ofGetFrameRate());

	ImGui::SliderFloat("v", &_v, 0.01f, 2.0f);
	ImGui::Checkbox("_check", &_check);
	ImGui::SliderFloat("theta", &_theta, -glm::pi<double>() * 0.5, glm::pi<double>() * 0.5);
	ImGui::SliderFloat("phi", &_phi, -glm::pi<double>(), glm::pi<double>());

	auto wp = ImGui::GetWindowPos();
	auto ws = ImGui::GetWindowSize();
	ofRectangle win(wp.x, wp.y, ws.x, ws.y);

	ImGui::End();
	ImGui::PopStyleColor();

	_imgui.end();

	if (win.inside(ofGetMouseX(), ofGetMouseY())) {
		_camera.disableMouseInput();
	}
	else {
		_camera.enableMouseInput();
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

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
