#include "ofApp.h"

static bool DRAW_ORANEG = true;

inline ofVec3f toOf(rt::Vec3 v) {
	return ofVec3f(v.x, v.y, v.z);
}

inline void drawAABB(rt::AABB aabb) {
	auto sR = aabb.size();
	auto cR = aabb.center();
	ofNoFill();
	ofDrawBox(cR.x, cR.y, cR.z, sR.x, sR.y, sR.z);
	ofFill();
}
inline void drawOBB(rt::OBB obb) {
	ofNoFill();
	ofPushMatrix();

	ofTranslate(obb.ac.x, obb.ac.y, obb.ac.z);
	rt::Mat3 m(obb.a[0], obb.a[1], obb.a[2]);
	ofMatrix4x4 ofm;
	ofm.set(glm::value_ptr(rt::Mat4(m)));
	ofMultMatrix(ofm);
	auto size = obb.h * 2.0;
	ofDrawBox(0.0f, 0.0f, 0.0f, size.x, size.y, size.z);

	ofPopMatrix();
	ofFill();
}
// ç≈Ç‡äÓñ{ÇÃBVHè’ìÀîªíË
inline void drawBVHOBB(int viewDepth, const rt::BVHNode &node, const rt::BezierEntity *beziers, int depth) {
	if (node.is<std::unique_ptr<rt::BVHOBBBranch>>()) {
		const std::unique_ptr<rt::BVHOBBBranch> &branch = node.get<std::unique_ptr<rt::BVHOBBBranch>>();
		if (viewDepth == depth) {
			drawOBB(branch->obb_L);
			drawOBB(branch->obb_R);
		}
		drawBVHOBB(viewDepth, branch->lhs, beziers, depth + 1);
		drawBVHOBB(viewDepth, branch->rhs, beziers, depth + 1);
	}
	if (node.is<rt::BVHLeaf>()) {
		auto drawBezier = [=](rt::BezierQuadratic3D bz) {
			ofNoFill();
			ofPolyline line;
			int N = 40;
			for (int i = 0; i < N; ++i) {
				float t = (float)i / (N - 1);
				auto p = bz.evaluate(t);

				line.addVertex(p.x, p.y, p.z);

				auto tangent = glm::normalize(bz.tangent(t));
				auto from_bxdf = rt::from_bxdf_basis_transform(tangent);
			}

			line.draw();

			ofFill();

			ofDrawSphere(toOf(bz[0]), 0.002f);
			ofDrawSphere(toOf(bz[1]), 0.002f);
			ofDrawSphere(toOf(bz[2]), 0.002f);
		};
		//static int k = 0;
		//if (k++ % 2 == 0) {
		//	ofSetColor(ofColor::orange);
		//}
		//else {
		//	ofSetColor(ofColor::blue);
		//}
		//if (DRAW_ORANEG == false) {
		//	ofSetColor(ofColor::blue);
		//}
		const rt::BVHLeaf &leaf = node.get<rt::BVHLeaf>();
		for (int i = 0; i < leaf.bezierIndices.size(); ++i) {
			auto index = leaf.bezierIndices[i];
			drawBezier(beziers[index].bezier);
		}
	}
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
		}

		line.draw();

		ofFill();
	};

	for (int i = 0; i < _leafs[_index].beizers.size(); ++i) {
		auto bezier = _leafs[_index].beizers[i];
		drawBezier(bezier.bezier);
	}

	// 
	{
		rt::OnlineCovarianceMatrix3x3 onlineCovMat3;
		rt::OnlineMeanT<rt::Vec3> center;
		for (int i = 0; i < _leafs[_index].beizers.size(); ++i) {
			auto bezier = _leafs[_index].beizers[i];

			int N = 10;
			for (int j = 0; j < N; ++j) {
				float t = (float)j / (N - 1);
				auto p = bezier.bezier.evaluate(t);
				onlineCovMat3.addSample(p);
				center.addSample(p);
			}
		}

		auto m = onlineCovMat3.sampleCovarianceMatrix();
		rt::Vec3 xaxis, yaxis, zaxis;
		std::tie(xaxis, yaxis, zaxis) = rt::PCA(m);

		ofSetColor(255, 0, 0);
		ofDrawLine(toOf(center.mean()), toOf(center.mean() + xaxis));
		ofSetColor(0, 0, 255);
		ofDrawLine(toOf(center.mean()), toOf(center.mean() + yaxis));
		ofSetColor(0, 255, 0);
		ofDrawLine(toOf(center.mean()), toOf(center.mean() + zaxis));

		rt::OnlineAABB onlineAABB;
		rt::OnlineOBB onlineOBB({ xaxis, yaxis, zaxis });

		for (int i = 0; i < _leafs[_index].beizers.size(); ++i) {
			auto bezier = _leafs[_index].beizers[i];

			int N = 100;
			for (int j = 0; j < N; ++j) {
				float t = (float)j / (N - 1);
				auto p = bezier.bezier.evaluate(t);
				onlineOBB.addSample(p);
				onlineAABB.addSample(p);
			}
		}
		rt::AABB aabb = onlineAABB.aabb();
		rt::OBB obb = onlineOBB.obb();

		//rt::OBB obb;
		//obb.ac = rt::Vec3();
		//obb.a = { xaxis , yaxis , zaxis };
		//obb.h = {0.5, 1.0, 1.5};

		ofSetColor(ofColor::orange);
		drawOBB(obb);


		ofSetColor(ofColor::red);
		drawAABB(aabb);

		// rt::AABB aabb(rt::Vec3(-1.0), rt::Vec3(1.0));

		//int NRay = 300;
		//for (int i = 0; i < NRay; ++i) {
		//	double s = (double)i / NRay;
		//	auto o = rt::Vec3(glm::mix(-2.0, 2.0, s), _ray_y, _ray_z);
		//	auto d = rt::Vec3(0.0, 0.0, -1.0);

		//	// auto tmin = rt::intersect_obb(rt::Ray(o, d), obb)
		//	// auto tmin = rt::intersect_aabb(rt::Ray(o, d), rt::Vec3(1.0) / d, aabb)
		//	if (auto tmin = rt::intersect_obb(rt::Ray(o, d), obb)) {
		//		ofDrawSphere(toOf(o), 0.01f);
		//		ofDrawLine(toOf(o), toOf(o + d * (*tmin)));
		//	}
		//	else {
		//		ofSetColor(ofColor::orange);
		//		ofDrawSphere(toOf(o), 0.01f);
		//		ofDrawLine(toOf(o), toOf(o + d * 5.0));
		//	}
		//}
	}

	if (_hasNode) {
		ofSetColor(ofColor::orange);
		drawBVHOBB(_viewDepth, _node, _beziers.data(), 0);
	}

	_camera.end();

	_imgui.begin();

	ImGui::PushStyleColor(ImGuiCol_WindowBg, ofVec4f(0.2f, 0.2f, 0.5f, 0.5f));
	ImGui::SetNextWindowPos(ofVec2f(500, 30), ImGuiSetCond_Once);
	ImGui::SetNextWindowSize(ofVec2f(500, 600), ImGuiSetCond_Once);

	ImGui::Begin("Config Panel");
	ImGui::Text("fps: %.2f", ofGetFrameRate());

	ImGui::SliderInt("index", &_index, 0, _leafs.size() - 1);

	ImGui::InputFloat("ray y", &_ray_y);
	ImGui::InputFloat("ray z", &_ray_z);

	ImGui::Checkbox("DRAW_ORANEG", &DRAW_ORANEG);

	//if (ImGui::Button("obb bvh")) {
	//	_beziers.clear();
	//	for (int i = 0; i < _leafs[_index].beizers.size(); ++i) {
	//		auto bezier = _leafs[_index].beizers[i];

	//		rt::BezierEntity e;
	//		e.bezier = bezier.bezier;
	//		e.radius = bezier.radius;
	//		_beziers.push_back(e);
	//	}


	//	std::vector<int> bezierIndices;
	//	for (int i = 0; i < _beziers.size(); ++i) {
	//		bezierIndices.push_back(i);
	//	}

	//	rt::AABB aabb;
	//	for (int i = 0; i < bezierIndices.size(); ++i) {
	//		auto bezierEntity = _beziers[bezierIndices[i]];
	//		auto bbox = bezierEntity.bezier.boundingBoxConvexhull();
	//		bbox.expand(bezierEntity.radius);
	//		aabb.expand(bbox);
	//	}

	//	rt::Xor random;
	//	_node = build_tree_obb(aabb, bezierIndices, _beziers.data(), &random, 0);
	//	_hasNode = true;
	//}

	ImGui::InputInt("_viewDepth", &_viewDepth);

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
