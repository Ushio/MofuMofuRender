#include "ofApp.h"

inline ofVec3f toOf(rt::Vec3 v) {
	return ofVec3f(v.x, v.y, v.z);
}
const int kSize = 512;


//--------------------------------------------------------------
void ofApp::setup() {

	_imgui.setup();

	_camera.setNearClip(0.1f);
	_camera.setFarClip(100.0f);
	_camera.setDistance(5.0f);

	_image.allocate(kSize, kSize, OF_IMAGE_GRAYSCALE);

	//for (int i = 0; i < 100; ++i) {
	//	double x = -2.0 * glm::pi<double>() + i * 0.1;
	//	double n = std::remainder(x, glm::two_pi<double>());
	//	printf("%.2f -> %.2f\n", x, n);
	//}
}

//--------------------------------------------------------------
void ofApp::update() {
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
	//ofPushMatrix();
	//ofRotateZ(90.0f);
	//ofSetColor(255);
	//ofDrawGridPlane(1.0f);
	//ofPopMatrix();

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

	double kCylinderRadius = 0.05;
	ofPushMatrix();
	ofRotateZ(90);
	ofSetColor(255);
	ofNoFill();
	ofDrawCylinder(kCylinderRadius, 3.0f);
	ofFill();
	ofPopMatrix();

	using namespace rt;

#if 0
	// Mp
	ofSetColor(ofColor::orange);
	ofSetLineWidth(3);
	ofDrawLine(ofVec3f(), ofVec3f(0, 0, 1).rotated(_thetaO, ofVec3f(0, 1, 0)));
	ofSetLineWidth(1);

	ofVec3f wo = ofVec3f(0, 0, 1).rotated(_thetaO, ofVec3f(0, 1, 0));
	double sinThetaO = wo.x;
	double cosThetaO = SafeSqrt(1.0 - Sqr(sinThetaO));

	double v = rt::betam_to_v(_betaM)[_p];

	ofPolyline polyline;
	int N = 1000;
	for (int i = 0; i < N; ++i) {
		// float theta_i = ofMap(i, 0, N, -180, 180);
		float theta_i = ofMap(i, 0, N, -90, 90);
		ofVec3f wi = ofVec3f(0, 0, 1.0f).rotated(theta_i, ofVec3f(0, 1, 0));

		double sinThetaI = wi.x;
		double cosThetaI = SafeSqrt(1.0 - Sqr(sinThetaI));
		double y = Mp(sinThetaI, cosThetaI, sinThetaO, cosThetaO, v);
		polyline.addVertex(wi * y);
	}
	ofSetColor(255);
	polyline.draw();
#endif

#if 0
	// サンプリングが確立密度関数に従っているか
	{
		std::array<int, 50> histgram = {};

		int NSample = 1000000;
		static Xor random;
		double v = rt::betam_to_v(_betaM)[_p];
		for (int i = 0; i < NSample; ++i) {
			double eps1 = random.uniform();
			double eps2 = random.uniform();
			// Importance Sampling for Physically-Based Hair Fiber Models ver
			// double u = v * glm::log(glm::exp(1.0 / v) - 2.0 * eps1 * glm::sinh(1.0 / v));

			// Numerically stable sampling of the von Mises Fisher distribution on S2 (and other tricks) ver
			// double u = v * glm::log(glm::exp(-1.0 / v) + 2.0 * eps1 * glm::sinh(1.0 / v));

			// Numerically stable sampling of the von Mises Fisher distribution on S2 (and other tricks) ver
			// avoids overflow ver.

			double u = 1.0 + v * glm::log(eps1 + (1.0 - eps1) * glm::exp(-2.0 / v));

			double theta_cone = -glm::asin(sinThetaO);
			double theta_tap = glm::pi<double>() * 0.5 - theta_cone;
			double theta_i = glm::asin(u * glm::cos(theta_tap) + glm::sqrt(1.0 - u * u) * glm::cos(glm::two_pi<double>() * eps2) * glm::sin(theta_tap));
			
			// 等価である
			// double theta_tap = -glm::pi<double>() * 0.5 + theta_cone;
			// double theta_i = glm::asin(u * glm::cos(theta_tap) - glm::sqrt(1.0 - u * u) * glm::cos(glm::two_pi<double>() * eps2) * glm::sin(theta_tap));

			static rt::Remap toIndex(-glm::pi<double>() * 0.5, glm::pi<double>() * 0.5, 0, histgram.size());
			int index = toIndex(theta_i);
			index = glm::clamp(index, 0, (int)histgram.size() - 1);
			histgram[index]++;
		}

		ofPolyline poly;
		double deltaTheta = glm::pi<double>() / histgram.size();
		for (int i = 0; i < histgram.size(); ++i) {
			static rt::Remap toTheta(0, histgram.size(), -glm::pi<double>() * 0.5, glm::pi<double>() * 0.5);
			double theta_i = toTheta(i + 0.5);
			double P = (double)histgram[i] / (double)NSample;
			double similarMp = P / (deltaTheta * glm::cos(theta_i));

			ofVec3f wi = ofVec3f(0, 0, 1.0f).rotated(glm::degrees(theta_i), ofVec3f(0, 1, 0));
			poly.addVertex(wi * similarMp);
		}

		ofSetColor(ofColor::orange);
		poly.draw();
	}
#endif

#if 1
	// Np
	double phiO = glm::radians(_phiO);
	Vec3 wo(0.0, glm::cos(phiO), glm::sin(phiO));

	Vec3 h_dir = glm::rotateX(wo, glm::radians(-90.0));
	double h = _h;

	ofSetColor(ofColor::orange);
	ofSetLineWidth(3);
	ofDrawLine(toOf(h_dir * h * kCylinderRadius), toOf(wo + h_dir * h * kCylinderRadius));
	ofSetLineWidth(1);

	{
		double sinThetaO = glm::sin(0.0); // 0
		double cosThetaO = glm::cos(0.0); // 1

		ofPolyline poly;

		double eta = 1.55;
		double s = betan_to_s(_betaN);
		int N = 1000;
		for (int i = 0; i < N; ++i) {
			static rt::Remap toPhiI(0, N, -glm::pi<double>(), glm::pi<double>());
			double phiI = toPhiI(i);
			double phi = phiI - phiO;

			double gammaO = SafeASin(h);
			double etap = glm::sqrt(eta * eta - Sqr(sinThetaO)) / cosThetaO;
			double sinGammaT = h / etap;
			double cosGammaT = SafeSqrt(1 - Sqr(sinGammaT));
			double gammaT = SafeASin(sinGammaT);
			double np = Np(phi, _p, s, gammaO, gammaT);

			Vec3 wo(0.0, glm::cos(phiI), glm::sin(phiI));
			poly.addVertex(toOf(wo * np));
		}

		ofSetColor(255);
		poly.draw();
	}
#endif

#if 0
	// Logistic
	{
		double s = betan_to_s(_betaN);
		double a = -glm::pi<double>();
		double b = glm::pi<double>();

		ofPolyline poly;

		int N = 500;
		for (int i = 0; i < N; ++i) {
			static rt::Remap toX(0, N, a, b);
			double x = toX(i);
			double value = rt::TrimmedLogistic(x, s, a, b);
			poly.addVertex(ofVec3f(x, value, 0.0));
		}
		ofSetColor(255);
		poly.draw();
	}
	{
		static rt::Xor random;

		double s = betan_to_s(_betaN);
		double a = -glm::pi<double>();
		double b = glm::pi<double>();

		ofPolyline poly;

		std::array<int, 100> histgram = {};
		int NSample = 1000000;
		for (int i = 0; i < NSample; ++i) {
			static rt::Remap toIndex(a, b, 0, histgram.size());

			double sample = rt::sampleTrimmedLogistic(random.uniform(), s, a, b);
			int index = toIndex(sample);
			index = glm::clamp(index, 0, (int)histgram.size() - 1);
			histgram[index]++;
		}

		for (int i = 0; i < histgram.size(); ++i) {
			double p = (double)histgram[i] / NSample;
			double similarTrimmedLogistic = p / ((b - a) / histgram.size());
			static rt::Remap toX(0, histgram.size(), a, b);
			double x = toX(i + 0.5);
			poly.addVertex(ofVec3f(x, similarTrimmedLogistic, 0.0));
			rt::TrimmedLogistic(x, s, a, b);
		}
		ofSetColor(ofColor::orange);
		poly.draw();
	}
#endif

#if 1
	{
		static rt::Xor random;

		double s = betan_to_s(_betaN);
		double a = -glm::pi<double>();
		double b = glm::pi<double>();

		double sinThetaO = glm::sin(0.0); // 0
		double cosThetaO = glm::cos(0.0); // 1
		double eta = 1.55;

		ofPolyline poly;

		std::array<int, 100> histgram = {};
		int NSample = 200000;
		for (int i = 0; i < NSample; ++i) {
			static rt::Remap toIndex(a, b, 0, histgram.size());

			double gammaO = SafeASin(h);
			double etap = glm::sqrt(eta * eta - Sqr(sinThetaO)) / cosThetaO;
			double sinGammaT = h / etap;
			double cosGammaT = SafeSqrt(1 - Sqr(sinGammaT));
			double gammaT = SafeASin(sinGammaT);

			double phiI = rt::sampleNp(random.uniform(), phiO, _p, s, gammaO, gammaT);
			
			int index = toIndex(phiI);
			index = glm::clamp(index, 0, (int)histgram.size() - 1);
			histgram[index]++;
		}

		for (int i = 0; i < histgram.size(); ++i) {
			double p = (double)histgram[i] / NSample;
			double similarNp = p / ((b - a) / histgram.size());
			static rt::Remap toPhiI(0, histgram.size(), a, b);
			double phiI = toPhiI(i + 0.5);
			Vec3 wi(0.0, glm::cos(phiI), glm::sin(phiI));

			poly.addVertex(toOf(wi * similarNp));
		}
		ofSetColor(ofColor::orange);
		ofSetLineWidth(2);
		poly.draw();
		ofSetLineWidth(1);
	}
#endif
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
	ImGui::SliderFloat("_betaM", &_betaM, 0, 1);
	ImGui::SliderInt("_p", &_p, 0, 3);
	ImGui::SliderFloat("_phiO", &_phiO, -180, 180);
	ImGui::SliderFloat("_h", &_h, -1, 1);
	ImGui::SliderFloat("_betaN", &_betaN, 0, 1);
	
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
