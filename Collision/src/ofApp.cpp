#include "ofApp.h"

//rt::Ray ray1;
//rt::Ray ray2;
//
//inline float ClosestPtSegmentSegment(rt::Vec3 p1, rt::Vec3 d1, rt::Vec3 p2, rt::Vec3 d2, double &s, double &t, rt::Vec3 &c1, rt::Vec3 &c2)
//{
//	using namespace rt;
//	Vec3 r = p1 - p2;
//	float a = glm::dot(d1, d1); // 線分S1の距離の平方、常に非負
//	float e = glm::dot(d2, d2); // 線分S2の距離の平方、常に非負
//	float f = glm::dot(d2, r);
//
//	// ここから一般的な縮退の場合を開始
//	float b = glm::dot(d1, d2);
//	float denom = a*e - b*b; // 常に非負
//
//	float c = glm::dot(d1, r);
//	// 線分が平行でない場合、L1上のL2に対する最近接点を計算、そして
//	// 線分S1に対してクランプ。そうでない場合は任意s(ここでは0)を選択
//	if (denom != 0.0f) {
//		// s = Clamp((b*f - c*e) / denom, 0.0f, 1.0f);
//		s = (b*f - c*e) / denom;
//	}
//	else {
//		s = 0.0f;
//	}
//	// L2上のS1(s)に対する最近接点を以下を用いて計算
//	// t = Dot((P1+D1*s)-P2,D2) / Dot(D2,D2) = (b*s + f) / e
//	t = (b*s + f) / e;
//
//	c1 = p1 + d1 * s;
//	c2 = p2 + d2 * t;
//	return glm::dot(c1 - c2, c1 - c2);
//}


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

	double radius = 0.1;

	auto drawBezier = [=](rt::BezierQuadratic3D bz) {
		{
			ofSetColor(ofColor::white);
			ofDrawSphere(bz[0].x, bz[0].y, bz[0].z, 0.02f);

			ofSetColor(ofColor::red);
			ofDrawSphere(bz[2].x, bz[2].y, bz[2].z, 0.02f);

			ofSetColor(ofColor::orange);
			ofDrawSphere(bz[1].x, bz[1].y, bz[1].z, 0.01f);
			ofLine(bz[1].x, bz[1].y, bz[1].z, bz[0].x, bz[0].y, bz[0].z);
			ofLine(bz[1].x, bz[1].y, bz[1].z, bz[2].x, bz[2].y, bz[2].z);
		}
		ofSetColor(ofColor::orange);

		ofNoFill();

		ofPolyline line;
		int N = 40;
		for (int i = 0; i < N; ++i) {
			float t = (float)i / (N - 1);
			auto p = bz.evaluate(t);

			line.addVertex(p.x, p.y, p.z);

			auto tangent = glm::normalize(bz.tangent(t));
			auto from_bxdf = rt::from_bxdf_basis_transform(tangent);

			ofPushMatrix();
			ofTranslate(p.x, p.y, p.z);
			ofMultMatrix(glm::value_ptr(glm::mat4(from_bxdf)));
			ofDrawCircle(0, 0, radius);
			ofPopMatrix();
		}

		line.draw();

		ofFill();
	};
	drawBezier(_bezierQuadratic);
	int NRay = 300;
	for (int i = 0; i < NRay; ++i) {
		double s = (double)i / NRay;
		auto o = rt::Vec3(glm::mix(-2.0, 2.0, s), _ray_y, _ray_z);
		auto d = rt::Vec3(0.0, 0.0, -1.0);

		auto projection = rt::ray_projection(o, d);
		auto bezier = _bezierQuadratic.transform(projection);

		double tmin = std::numeric_limits<double>::max();
		rt::CurveIntersection intersection;
		if (rt::intersect_bezier(15, radius, radius * radius, bezier, bezier, 0.0, 1.0, &tmin, &intersection)) {
			if (intersection.h < 0.0) {
				ofSetColor(ofColor::red * glm::abs(intersection.h));
			}
			else {
				ofSetColor(ofColor::blue * glm::abs(intersection.h));
			}
			ofDrawSphere(toOf(o), 0.01f);
			ofDrawLine(toOf(o), toOf(o + d * tmin));
		} else {
			ofSetColor(ofColor::orange);
			ofDrawSphere(toOf(o), 0.01f);
			ofDrawLine(toOf(o), toOf(o + d * 5.0));
		}
	}

	//ofSetColor(ofColor::orange);
	//ofDrawSphere(toOf(ray1.o), 0.05f);
	//ofDrawLine(toOf(ray1.o - ray1.d * 5.0), toOf(ray1.o + ray1.d * 5.0));
	//
	//ofSetColor(ofColor::red);
	//ofDrawSphere(toOf(ray2.o), 0.05f);
	//ofDrawLine(toOf(ray2.o - ray2.d * 5.0), toOf(ray2.o + ray2.d * 5.0));

	//double s, t;
	//rt::Vec3 p1, p2;
	//ClosestPtSegmentSegment(ray1.o, ray1.d, ray2.o, ray2.d, s, t, p1, p2);

	//ofSetColor(ofColor::green);
	//ofDrawLine(toOf(p1), toOf(p2));

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

	ImGui::InputFloat("ray y", &_ray_y);
	ImGui::InputFloat("ray z", &_ray_z);

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
		static rt::Xor random;
		auto r = []() {
			return rt::Vec3(random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)) * 2.0;
		};
		_bezierQuadratic = rt::BezierQuadratic3D(r(), r(), r());

		//ray1 = rt::Ray(r(), rt::uniform_on_unit_sphere(&random));
		//ray2 = rt::Ray(r(), rt::uniform_on_unit_sphere(&random));
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
