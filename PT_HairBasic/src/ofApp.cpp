#include "ofApp.h"

inline ofVec3f toOf(rt::Vec3 v) {
	return ofVec3f(v.x, v.y, v.z);
}

inline ofPixels toOf(const rt::Image &image) {
	ofPixels pixels;
	pixels.allocate(image.width(), image.height(), OF_IMAGE_COLOR);
	uint8_t *dst = pixels.getPixels();

	double scale = 1.0;
	for (int y = 0; y < image.height(); ++y) {
		for (int x = 0; x < image.width(); ++x) {
			int index = y * image.width() + x;
			auto px = *image.pixel(x, y);
			auto L = px.color / (double)px.sample;
			dst[index * 3 + 0] = (uint8_t)glm::clamp(glm::pow(L.x * scale, 1.0 / 2.2) * 255.0, 0.0, 255.99999);
			dst[index * 3 + 1] = (uint8_t)glm::clamp(glm::pow(L.y * scale, 1.0 / 2.2) * 255.0, 0.0, 255.99999);
			dst[index * 3 + 2] = (uint8_t)glm::clamp(glm::pow(L.z * scale, 1.0 / 2.2) * 255.0, 0.0, 255.99999);
		}
	}
	return pixels;
}

//--------------------------------------------------------------
void ofApp::setup() {
	_wholeSW = std::shared_ptr<rt::Stopwatch>(new rt::Stopwatch());

	static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
	static plog::RollingFileAppender<plog::TxtFormatter> fileAppender(ofToDataPath("../../log.txt").c_str());
	plog::init(plog::debug, &consoleAppender).addAppender(&fileAppender);

#if NO_WINDOW == 0
	_imgui.setup();

	_camera.setNearClip(0.1f);
	_camera.setFarClip(100.0f);
	_camera.setDistance(5.0f);
#else
#endif

	_sw = std::shared_ptr<rt::Stopwatch>(new rt::Stopwatch());
	_scene = scene_conelbox();

	//{
	//	std::vector<rt::BezierQuadratic3D> beziers;
	//	rt::Xor random;
	//	for (int i = 0; i < 5000; ++i) {
	//		//auto p = rt::uniform_in_unit_circle(&random);
	//		//rt::BezierQuadratic3D bz(rt::Vec3(p.x, -1.0, p.y), rt::Vec3(0.0, random.uniform(0.2, 0.0), 0.0), rt::Vec3(-p.x, -1.0, -p.y));
	//		//beziers.push_back(bz);

	//		auto p = rt::uniform_in_unit_circle(&random);
	//		auto p1 = rt::Vec3(p.x, -1.0, p.y);
	//		auto cp = p1 + rt::Vec3(random.uniform(-0.1, 0.1), 0.3, random.uniform(-0.1, 0.1));
	//		auto p2 = cp + rt::Vec3(random.uniform(-0.1, 0.1), 0.3, random.uniform(-0.1, 0.1));

	//		rt::BezierQuadratic3D bz(p1, cp, p2);

	//		rt::BezierEntity e;
	//		e.bezier = bz;
	//		e.sigma_a = rt::Vec3();
	//		e.radius = 0.005;
	//		beziers.push_back(bz);
	//	}
	//	std::shared_ptr<rt::BezierSceneElement> element(new rt::BezierSceneElement(beziers));
	//	_scene = _scene->addElement(element);
	//}

	{
		std::vector<rt::BezierEntity> beziers;
		rt::Xor random;
		for (int i = 0; i < 1000; ++i) {
			auto p = rt::uniform_in_unit_circle(&random);
			auto p1 = rt::Vec3(p.x, -1.0, p.y);
			auto cp = p1 + rt::Vec3(random.uniform(-0.1, 0.1), 0.2, random.uniform(-0.1, 0.1));
			auto p2 = cp + rt::Vec3(random.uniform(-0.1, 0.1), 0.2, random.uniform(-0.1, 0.1));

			rt::BezierQuadratic3D bz(p1, cp, p2);

			rt::BezierEntity e;
			e.bezier = bz;
			e.sigma_a = rt::Vec3(random.uniform(0.0, 0.8), random.uniform(0.0, 0.8), random.uniform(0.0, 0.8));
			e.radius = random.uniform(0.003, 0.008);
			beziers.push_back(e);
		}
		std::shared_ptr<rt::BezierBVHSceneElement> element(new rt::BezierBVHSceneElement(beziers));
		_scene = _scene->addElement(element);
	}

	_pt = std::shared_ptr<rt::PathTracer>(new rt::PathTracer(_scene));

	LOG_DEBUG << "render begin...";
}

//--------------------------------------------------------------
void ofApp::update(){
	rt::Stopwatch sw;
	_pt->step();
	_stepStats.addSample(sw.elapsed());

	LOG_DEBUG << "step: " << sw.elapsed();
	
#if NO_WINDOW == 0
	//if (ofGetFrameNum() % 30 == 0) {
	//	_image.setFromPixels(toOf(_pt->_image));
	//}
	_image.setFromPixels(toOf(_pt->_image));
#else
	if (_sw->elapsed() > 29.0) {
		static int index = 0;

		_sw = std::shared_ptr<rt::Stopwatch>(new rt::Stopwatch());

		// output
		auto image = toOf(_pt->_image);
		char name[256];
		sprintf(name, "../../output_images/%03d.png", index++);
		ofSaveImage(image, name);

		printf("saved %03d.png, %.1fs\n", index - 1, _wholeSW->elapsed());
	}

	if (_wholeSW->elapsed() + _stepStats.avarage() > 60.0 * 4.0 + 33.0) {
		std::exit(0);
	}
#endif
}

//--------------------------------------------------------------
void ofApp::draw(){
#if NO_WINDOW
	return;
#endif

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

	_camera.end();

	ofDisableDepthTest();
	ofSetColor(255);

	_image.draw(0, 0);

	_imgui.begin();

	ImGui::PushStyleColor(ImGuiCol_WindowBg, ofVec4f(0.2f, 0.2f, 0.5f, 0.5f));
	ImGui::SetNextWindowPos(ofVec2f(500, 30), ImGuiSetCond_Once);
	ImGui::SetNextWindowSize(ofVec2f(500, 600), ImGuiSetCond_Once);

	ImGui::Begin("Config Panel");
	ImGui::Text("fps: %.2f", ofGetFrameRate());
	ImGui::Text("time: %.2f", _sw->elapsed());
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
