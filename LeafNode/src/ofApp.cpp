#include "ofApp.h"


inline ofVec3f toOf(rt::Vec3 v) {
	return ofVec3f(v.x, v.y, v.z);
}

//--------------------------------------------------------------
void ofApp::setup() {
	_imgui.setup();

	_camera.setNearClip(0.1f);
	_camera.setFarClip(100.0f);
	_camera.setDistance(5.0f);

	// _image.allocate(kSize, kSize, OF_IMAGE_GRAYSCALE);
	FILE *fp = fopen(ofToDataPath("bvh.txt").c_str(), "r");
	bool hasStream = true;
	for (int i = 0; hasStream; ++i) {
		BezierLeaf leaf;
		for (int j = 0; ; ++j) {
			char str[256];
			if (fgets(str, 256, fp) == nullptr) {
				hasStream = false;
				break;
			}
			if (str[0] == 'l') {
				break;
			}

			BeizerFur fur;
			std::array<rt::Vec3, 3> ps;
			sscanf(str, "%lf/%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &fur.radius, 
				&ps[0].x, &ps[0].y, &ps[0].z,
				&ps[1].x, &ps[1].y, &ps[1].z,
				&ps[2].x, &ps[2].y, &ps[2].z);
			fur.bezier = rt::BezierQuadratic3D(ps[0], ps[1], ps[2]);
			leaf.beizers.push_back(fur);
		}
		if (leaf.beizers.empty()) {
			continue;
		}

		_leafs.push_back(leaf);
	}

	fclose(fp);
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
	auto drawBezier = [=](rt::BezierQuadratic3D bz) {
		//{
		//	ofSetColor(ofColor::white);
		//	ofDrawSphere(bz[0].x, bz[0].y, bz[0].z, 0.02f);

		//	ofSetColor(ofColor::red);
		//	ofDrawSphere(bz[2].x, bz[2].y, bz[2].z, 0.02f);

		//	ofSetColor(ofColor::orange);
		//	ofDrawSphere(bz[1].x, bz[1].y, bz[1].z, 0.01f);
		//	ofLine(bz[1].x, bz[1].y, bz[1].z, bz[0].x, bz[0].y, bz[0].z);
		//	ofLine(bz[1].x, bz[1].y, bz[1].z, bz[2].x, bz[2].y, bz[2].z);
		//}
		//ofSetColor(ofColor::orange);

		ofNoFill();
		ofSetColor(ofColor::orange);
		ofPolyline line;
		int N = 40;
		for (int i = 0; i < N; ++i) {
			float t = (float)i / (N - 1);
			auto p = bz.evaluate(t);

			line.addVertex(p.x, p.y, p.z);

			auto tangent = glm::normalize(bz.tangent(t));
			auto from_bxdf = rt::from_bxdf_basis_transform(tangent);
			/*
			ofPushMatrix();
			ofTranslate(p.x, p.y, p.z);
			ofMultMatrix(glm::value_ptr(glm::mat4(from_bxdf)));
			ofDrawCircle(0, 0, radius);
			ofPopMatrix();
			*/
		}

		line.draw();

		ofFill();
	};

	for (int i = 0; i < _leafs[_index].beizers.size(); ++i) {
		auto bezier = _leafs[_index].beizers[i];
		drawBezier(bezier.bezier);
	}

	_camera.end();

	_imgui.begin();

	ImGui::PushStyleColor(ImGuiCol_WindowBg, ofVec4f(0.2f, 0.2f, 0.5f, 0.5f));
	ImGui::SetNextWindowPos(ofVec2f(500, 30), ImGuiSetCond_Once);
	ImGui::SetNextWindowSize(ofVec2f(500, 600), ImGuiSetCond_Once);

	ImGui::Begin("Config Panel");
	ImGui::Text("fps: %.2f", ofGetFrameRate());

	ImGui::SliderInt("index", &_index, 0, _leafs.size() - 1);

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
