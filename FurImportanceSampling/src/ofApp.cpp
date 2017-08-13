#include "ofApp.h"


inline ofVec3f toOf(rt::Vec3 v) {
	return ofVec3f(v.x, v.y, v.z);
}
const int kSize = 512;

//
//inline double sampleCosTheta(double OCosTheta, double v, rt::PeseudoRandom *random) {
//	
//
//}


//--------------------------------------------------------------
void ofApp::setup() {
	_imgui.setup();

	_camera.setNearClip(0.1f);
	_camera.setFarClip(100.0f);
	_camera.setDistance(5.0f);

	_image.allocate(kSize, kSize, OF_IMAGE_GRAYSCALE);

}

//--------------------------------------------------------------
void ofApp::update(){
	//using namespace rt;
	//int size = _image.getWidth();

	//uint8_t *ptr = _image.getPixels().getPixels();
	//for (int x = 0; x < size; ++x) {
	//	double xn = (double)x / size;
	//	for (int y = 0; y < size; ++y) {
	//		uint8_t *px = ptr + y * size + x;

	//		double yn = (double)y / size;
	//		Vec3 ro(
	//			glm::mix(-2.0, 2.0, xn),
	//			glm::mix(2.0, -2.0, yn),
	//			-2.0
	//		);
	//		Vec3 rd(0.2, 0.0, 1.0);
	//		rd = glm::normalize(rd);

	//		auto pj = ray_projection(ro, rd);
	//		//auto bezier_local = _bezier3D.transform(pj);
	//		auto bezier_local = _bezierQuadratic.transform(pj);

	//		double tmin = 100000.0f;
	//		CurveIntersection intersection;
	//		if (intersect_ray_local(bezier_local, 0.05, &tmin, &intersection)) {
	//			double dp = glm::sqrt(intersection.hSq) / (0.05);
	//			// 
	//			//if ((int)(intersection.closest_t * 10.0) % 2 == 0) {
	//			//	px[0] = 255 - (uint8_t)glm::mix(0.0, 255.0, dp);
	//			//}
	//			//else {
	//			//	px[0] = (uint8_t)glm::mix(0.0, 255.0, dp);
	//			//}

	//			px[0] = (uint8_t)glm::mix(255.0, 0.0, dp);
	//			//px[0] = 255;
	//		}
	//		else {
	//			px[0] = 0;
	//		}
	//	}
	//}
	//_imageRayTrace.update();
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofEnableDepthTest();

	ofClear(0);
	_camera.begin();
	ofPushMatrix();
	ofRotateZ(90.0f);
	ofSetColor(255);
	ofDrawGridPlane(1.0f);
	ofPopMatrix();

	ofPushMatrix();
	ofDrawAxis(50);
	ofPopMatrix();

	//_scene->drawPreview([](const rt::Vec3 &p0, const rt::Vec3 &p1) {
	//	ofDrawLine(toOf(p0), toOf(p1));
	//});

	ofPushMatrix();
	ofSetLineWidth(3);
	ofDrawAxis(1);
	ofSetLineWidth(1);
	ofPopMatrix();

	ofPushMatrix();
	ofRotateZ(90);
	ofSetColor(255);
	ofNoFill();
	ofDrawCylinder(0.05f, 3.0f);
	ofFill();
	ofPopMatrix();

	// v=0.02, 固定_theta_o: −1 radian, −1.3 radians,	and −1.4 radians の時の図と合っている。
	using namespace rt;


	// theta
	ofSetColor(ofColor::orange);
	ofSetLineWidth(3);
	ofDrawLine(ofVec3f(), ofVec3f(0, 0, 1).rotated(_thetaO, ofVec3f(0, 1, 0)));
	ofSetLineWidth(1);

	ofVec3f wo = ofVec3f(0, 0, 1).rotated(_thetaO, ofVec3f(0, 1, 0));
	double sinThetaO = wo.x;
	double cosThetaO = SafeSqrt(1.0 - Sqr(sinThetaO));

	ofPolyline polyline;
	int N = 1000;
	for (int i = 0; i < N; ++i) {
		float theta_i = ofMap(i, 0, N, -180, 180);
		ofVec3f wi = ofVec3f(0, 0, 1.0f).rotated(theta_i, ofVec3f(0, 1, 0));

		double sinThetaI = wi.x;
		double cosThetaI = SafeSqrt(1.0 - Sqr(sinThetaI));
		double y = Mp(sinThetaI, cosThetaI, sinThetaO, cosThetaO, _v);
		polyline.addVertex(wi * y);
	}
	ofSetColor(255);
	polyline.draw();

	// 確かにエネルギー保存、確立密度関数が満たすべき性質を満たしている
	//{
	//	ofMesh mesh;
	//	mesh.setMode(OF_PRIMITIVE_POINTS);

	//	static Xor random;
	//	int NSample = 100000;
	//	double sum = 0.0;
	//	for (int i = 0; i < NSample; ++i) {
	//		double p = 1.0 / glm::pi<double>();
	//		double theta_i = random.uniform(-glm::pi<double>() * 0.5, glm::pi<double>() * 0.5);
	//		double cosThetaI = glm::cos(theta_i);
	//		double sinThetaI = glm::sin(theta_i);

	//		double mp = Mp(sinThetaI, cosThetaI, sinThetaO, cosThetaO, _v);
	//		sum += mp * cosThetaI / p;

	//		ofVec3f wi = ofVec3f(0, 0, 1.0f).rotated(glm::degrees(theta_i), ofVec3f(0, 1, 0));

	//		mesh.addVertex(wi * mp);
	//	}

	//	double E = sum / NSample;
	//	printf("%.2f\n", E);

	//	ofSetColor(255);
	//	mesh.draw();
	//}

	{
		// ofMesh mesh;
		// mesh.setMode(OF_PRIMITIVE_POINTS);

		std::array<int, 100> histgram = {};

		int NSample = 50000;
		static Xor random;
		double v = _v;
		for (int i = 0; i < NSample; ++i) {
			double eps1 = random.uniform();
			double eps2 = random.uniform();
			double u = v * glm::log(glm::exp(1.0 / v) - 2.0 * eps1 * glm::sinh(1.0 / v));
			double theta_cone = -glm::asin(sinThetaO);
			double theta_tap = glm::pi<double>() * 0.5 - theta_cone;
			double theta_i = glm::asin(u * glm::cos(theta_tap) + glm::sqrt(1.0 - u * u) * glm::cos(glm::two_pi<double>() * eps2) * glm::sin(theta_tap));

			// ofVec3f wi = ofVec3f(0, 0, 1.0f).rotated(glm::degrees(theta_i), ofVec3f(0, 1, 0));
			// mesh.addVertex(wi);

			int index = ofMap(theta_i, -glm::pi<double>() * 0.5, glm::pi<double>() * 0.5, 0, histgram.size());
			histgram[index]++;
		}
		// ofSetColor(255);
		// mesh.draw();

		ofPolyline poly;
		double deltaTheta = glm::pi<double>() / histgram.size();
		for (int i = 0; i < histgram.size(); ++i) {
			double theta_i = ofMap(i + 0.5, 0, histgram.size(), -glm::pi<double>() * 0.5, glm::pi<double>() * 0.5);
			double P = (double)histgram[i] / (double)NSample;
			double similarMp = P / (deltaTheta * glm::cos(theta_i));

			ofVec3f wi = ofVec3f(0, 0, 1.0f).rotated(glm::degrees(theta_i), ofVec3f(0, 1, 0));
			poly.addVertex(wi * similarMp);
		}

		ofSetColor(ofColor::orange);
		poly.draw();
	}


	_camera.end();

	ofDisableDepthTest();
	ofSetColor(255);

	// _image.draw(0, 0);

	_imgui.begin();

	ImGui::PushStyleColor(ImGuiCol_WindowBg, ofVec4f(0.2f, 0.2f, 0.5f, 0.5f));
	ImGui::SetNextWindowPos(ofVec2f(500, 30), ImGuiSetCond_Once);
	ImGui::SetNextWindowSize(ofVec2f(500, 600), ImGuiSetCond_Once);

	ImGui::Begin("Config Panel");
	ImGui::Text("fps: %.2f", ofGetFrameRate());

	ImGui::SliderFloat("_thetaO", &_thetaO, -90, 90);
	ImGui::InputFloat("_v", &_v);

	auto wp = ImGui::GetWindowPos();
	auto ws = ImGui::GetWindowSize();
	ofRectangle win(wp.x, wp.y, ws.x, ws.y);

	ImGui::End();
	ImGui::PopStyleColor();

	_imgui.end();

	if (win.inside(ofGetMouseX(), ofGetMouseY())) {
		_camera.disableMouseInput();
	} else {
		_camera.enableMouseInput();
	}

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == ' ') {

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
