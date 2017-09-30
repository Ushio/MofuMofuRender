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

inline std::shared_ptr<rt::Scene> scene_fromUnity() {
	using namespace rt;

	rt::CameraSetting cameraSetting;
	cameraSetting._imageWidth = 320;
	cameraSetting._imageHeight = 240;

	cameraSetting._eye = Vec3(1.951, 0.93, 1.19);
	cameraSetting._lookat = Vec3(1.26328, 0.4757957, 0.6236613);
	cameraSetting._up = Vec3(-0.4395704, 0.8813744, -0.1730812);
	cameraSetting._fov = 1.047198;

	std::vector<std::shared_ptr<rt::SceneElement>> sceneElements;

	// ‚±‚ê‚Í–³‘Ê‚È‚P–‡
	{
		std::vector<Vec3> vertices = {
			rt::Vec3(-0.5, 0.5, 0.0),
			rt::Vec3(0.5, 0.5, 0.0),
			rt::Vec3(-0.5,-0.5, 0.0),
			rt::Vec3(0.5, -0.5, 0.0)
		};
		std::vector<int> indices = {
			1, 0, 2,
			1, 2, 3
		};

		Transform transform = Transform(Vec3(0, 0, 0), glm::angleAxis(-1.265752, Vec3(0.53636, -0.3051372, 0.7868984)), Vec3(1, 1, 1));
		transform.apply(vertices);
		std::shared_ptr<rt::PolygonSceneElement> floor(new rt::PolygonSceneElement(vertices, indices,
			LambertianMaterial(rt::Vec3(0.0), rt::Vec3(0.75)),
			false
		));
		sceneElements.push_back(floor);
	}

	{
		std::vector<Vec3> vertices = {
			rt::Vec3(-0.5, 0.5, 0.0),
			rt::Vec3(0.5, 0.5, 0.0),
			rt::Vec3(-0.5,-0.5, 0.0),
			rt::Vec3(0.5, -0.5, 0.0)
		};
		std::vector<int> indices = {
			1, 0, 2,
			1, 2, 3
		};


		Transform transform = Transform(Vec3(0, -0.6, 0), glm::angleAxis(-1.570796, Vec3(1, 0, 0)), Vec3(10, 10, 10));
		transform.apply(vertices);
		std::shared_ptr<rt::PolygonSceneElement> floor(new rt::PolygonSceneElement(vertices, indices,
			LambertianMaterial(rt::Vec3(0.0), rt::Vec3(0.75)),
			false
		));
		sceneElements.push_back(floor);
	}

	{
		std::vector<Vec3> vertices = {
			rt::Vec3(-0.5, 0.5, 0.0),
			rt::Vec3(0.5, 0.5, 0.0),
			rt::Vec3(-0.5,-0.5, 0.0),
			rt::Vec3(0.5, -0.5, 0.0)
		};
		std::vector<int> indices = {
			1, 0, 2,
			1, 2, 3
		};
		Transform transform = Transform(Vec3(0.77, 2.3, 1.64), glm::angleAxis(-1.96795, Vec3(-1, 0, 0)), Vec3(1.990666, 1.990666, 1.990666));
		for (int i = 0; i < vertices.size(); ++i) {
			vertices[i] = transform * vertices[i];
		}
		std::shared_ptr<rt::PolygonSceneElement> light(new rt::PolygonSceneElement(vertices, indices,
			LambertianMaterial(rt::Vec3(10.0), rt::Vec3(0.75)),
			false
		));
		sceneElements.push_back(light);
	}

	// =============================
	{
		std::vector<Vec3> vertices = {
			rt::Vec3(0.5, -0.5, 0.5),
			rt::Vec3(-0.5, -0.5, 0.5),
			rt::Vec3(0.5, 0.5, 0.5),
			rt::Vec3(-0.5, 0.5, 0.5),
			rt::Vec3(0.5, 0.5, -0.5),
			rt::Vec3(-0.5, 0.5, -0.5),
			rt::Vec3(0.5, -0.5, -0.5),
			rt::Vec3(-0.5, -0.5, -0.5),
			rt::Vec3(0.5, 0.5, 0.5),
			rt::Vec3(-0.5, 0.5, 0.5),
			rt::Vec3(0.5, 0.5, -0.5),
			rt::Vec3(-0.5, 0.5, -0.5),
			rt::Vec3(0.5, -0.5, -0.5),
			rt::Vec3(0.5, -0.5, 0.5),
			rt::Vec3(-0.5, -0.5, 0.5),
			rt::Vec3(-0.5, -0.5, -0.5),
			rt::Vec3(-0.5, -0.5, 0.5),
			rt::Vec3(-0.5, 0.5, 0.5),
			rt::Vec3(-0.5, 0.5, -0.5),
			rt::Vec3(-0.5, -0.5, -0.5),
			rt::Vec3(0.5, -0.5, -0.5),
			rt::Vec3(0.5, 0.5, -0.5),
			rt::Vec3(0.5, 0.5, 0.5),
			rt::Vec3(0.5, -0.5, 0.5) };
		std::vector<int> indices = { 0,2,3,0,3,1,8,4,5,8,5,9,10,6,7,10,7,11,12,13,14,12,14,15,16,17,18,16,18,19,20,21,22,20,22,23 };

		Transform transform = Transform(Vec3(1.52, 0.28, -1.61), glm::angleAxis(double(-2.219941), Vec3(-0.0373482, 0.901526, -0.43111)), Vec3(0.893393, 0.8933936, 0.8933936));
		transform.apply(vertices);
		std::shared_ptr<rt::PolygonSceneElement> element(new rt::PolygonSceneElement(vertices, indices,
			LambertianMaterial(rt::Vec3(0.0), rt::Vec3(1, 0.2720588, 0.6787016)),
			false
		));
		sceneElements.push_back(element);
	}
	{
		std::vector<Vec3> vertices = {
			rt::Vec3(0.5, -0.5, 0.5),
			rt::Vec3(-0.5, -0.5, 0.5),
			rt::Vec3(0.5, 0.5, 0.5),
			rt::Vec3(-0.5, 0.5, 0.5),
			rt::Vec3(0.5, 0.5, -0.5),
			rt::Vec3(-0.5, 0.5, -0.5),
			rt::Vec3(0.5, -0.5, -0.5),
			rt::Vec3(-0.5, -0.5, -0.5),
			rt::Vec3(0.5, 0.5, 0.5),
			rt::Vec3(-0.5, 0.5, 0.5),
			rt::Vec3(0.5, 0.5, -0.5),
			rt::Vec3(-0.5, 0.5, -0.5),
			rt::Vec3(0.5, -0.5, -0.5),
			rt::Vec3(0.5, -0.5, 0.5),
			rt::Vec3(-0.5, -0.5, 0.5),
			rt::Vec3(-0.5, -0.5, -0.5),
			rt::Vec3(-0.5, -0.5, 0.5),
			rt::Vec3(-0.5, 0.5, 0.5),
			rt::Vec3(-0.5, 0.5, -0.5),
			rt::Vec3(-0.5, -0.5, -0.5),
			rt::Vec3(0.5, -0.5, -0.5),
			rt::Vec3(0.5, 0.5, -0.5),
			rt::Vec3(0.5, 0.5, 0.5),
			rt::Vec3(0.5, -0.5, 0.5) };
		std::vector<int> indices = { 0,2,3,0,3,1,8,4,5,8,5,9,10,6,7,10,7,11,12,13,14,12,14,15,16,17,18,16,18,19,20,21,22,20,22,23 };

		Transform transform = Transform(Vec3(-2.12, 0.23, -0.58), glm::angleAxis(double(-1.096531), Vec3(-0.7443157, 0.2764671, -0.6079147)), Vec3(0.3030529, 0.303053, 0.303053));
		transform.apply(vertices);
		std::shared_ptr<rt::PolygonSceneElement> element(new rt::PolygonSceneElement(vertices, indices,
			LambertianMaterial(rt::Vec3(0.0), rt::Vec3(0.2049695, 0.1397059, 1)),
			false
		));
		sceneElements.push_back(element);
	}
	{
		std::vector<Vec3> vertices = {
			rt::Vec3(0.5, -0.5, 0.5),
			rt::Vec3(-0.5, -0.5, 0.5),
			rt::Vec3(0.5, 0.5, 0.5),
			rt::Vec3(-0.5, 0.5, 0.5),
			rt::Vec3(0.5, 0.5, -0.5),
			rt::Vec3(-0.5, 0.5, -0.5),
			rt::Vec3(0.5, -0.5, -0.5),
			rt::Vec3(-0.5, -0.5, -0.5),
			rt::Vec3(0.5, 0.5, 0.5),
			rt::Vec3(-0.5, 0.5, 0.5),
			rt::Vec3(0.5, 0.5, -0.5),
			rt::Vec3(-0.5, 0.5, -0.5),
			rt::Vec3(0.5, -0.5, -0.5),
			rt::Vec3(0.5, -0.5, 0.5),
			rt::Vec3(-0.5, -0.5, 0.5),
			rt::Vec3(-0.5, -0.5, -0.5),
			rt::Vec3(-0.5, -0.5, 0.5),
			rt::Vec3(-0.5, 0.5, 0.5),
			rt::Vec3(-0.5, 0.5, -0.5),
			rt::Vec3(-0.5, -0.5, -0.5),
			rt::Vec3(0.5, -0.5, -0.5),
			rt::Vec3(0.5, 0.5, -0.5),
			rt::Vec3(0.5, 0.5, 0.5),
			rt::Vec3(0.5, -0.5, 0.5) };
		std::vector<int> indices = { 0,2,3,0,3,1,8,4,5,8,5,9,10,6,7,10,7,11,12,13,14,12,14,15,16,17,18,16,18,19,20,21,22,20,22,23 };

		Transform transform = Transform(Vec3(-1.75, 0.13, -2.32), glm::angleAxis(double(-2.219941), Vec3(-0.0373482, 0.901526, -0.43111)), Vec3(0.6561986, 0.6561989, 0.6561989));
		transform.apply(vertices);
		std::shared_ptr<rt::PolygonSceneElement> element(new rt::PolygonSceneElement(vertices, indices,
			LambertianMaterial(rt::Vec3(0.0), rt::Vec3(0, 0.9586205, 1)),
			false
		));
		sceneElements.push_back(element);
	}
	{
		std::vector<Vec3> vertices = {
			rt::Vec3(0.5, -0.5, 0.5),
			rt::Vec3(-0.5, -0.5, 0.5),
			rt::Vec3(0.5, 0.5, 0.5),
			rt::Vec3(-0.5, 0.5, 0.5),
			rt::Vec3(0.5, 0.5, -0.5),
			rt::Vec3(-0.5, 0.5, -0.5),
			rt::Vec3(0.5, -0.5, -0.5),
			rt::Vec3(-0.5, -0.5, -0.5),
			rt::Vec3(0.5, 0.5, 0.5),
			rt::Vec3(-0.5, 0.5, 0.5),
			rt::Vec3(0.5, 0.5, -0.5),
			rt::Vec3(-0.5, 0.5, -0.5),
			rt::Vec3(0.5, -0.5, -0.5),
			rt::Vec3(0.5, -0.5, 0.5),
			rt::Vec3(-0.5, -0.5, 0.5),
			rt::Vec3(-0.5, -0.5, -0.5),
			rt::Vec3(-0.5, -0.5, 0.5),
			rt::Vec3(-0.5, 0.5, 0.5),
			rt::Vec3(-0.5, 0.5, -0.5),
			rt::Vec3(-0.5, -0.5, -0.5),
			rt::Vec3(0.5, -0.5, -0.5),
			rt::Vec3(0.5, 0.5, -0.5),
			rt::Vec3(0.5, 0.5, 0.5),
			rt::Vec3(0.5, -0.5, 0.5) };
		std::vector<int> indices = { 0,2,3,0,3,1,8,4,5,8,5,9,10,6,7,10,7,11,12,13,14,12,14,15,16,17,18,16,18,19,20,21,22,20,22,23 };

		Transform transform = Transform(Vec3(0.12, 0.86, -1.8), glm::angleAxis(double(-2.916905), Vec3(-0.3259515, 0.4244604, -0.8447419)), Vec3(0.3650931, 0.3650928, 0.3650928));
		transform.apply(vertices);
		std::shared_ptr<rt::PolygonSceneElement> element(new rt::PolygonSceneElement(vertices, indices,
			LambertianMaterial(rt::Vec3(0.0), rt::Vec3(0.9204869, 1, 0.1764706)),
			false
		));
		sceneElements.push_back(element);
	}


	return std::shared_ptr<rt::Scene>(new rt::Scene(cameraSetting, sceneElements));
}
//--------------------------------------------------------------
void ofApp::setup() {
	_imgui.setup();

	_camera.setNearClip(0.1f);
	_camera.setFarClip(100.0f);
	_camera.setDistance(5.0f);

	_sw = std::shared_ptr<rt::Stopwatch>(new rt::Stopwatch());

	_scene = scene_conelbox();

	rt::Material meshMat = rt::LambertianMaterial(rt::Vec3(0.0), rt::Vec3(0.5));
	// rt::Material meshMat = rt::SpecularMaterial();
	//std::shared_ptr<rt::SphereSceneElement> element(new rt::SphereSceneElement(rt::Sphere(rt::Vec3(0.0, -0.6, 0.0), 0.4), meshMat));
	//_scene = _scene->addElement(element);
	rt::TruncatedCone cone;
	cone.p = rt::Vec3(0.0, -1.0, 0.0);
	cone.p_radius = 0.5f;
	cone.q = rt::Vec3(0.0, -0.8f, 0.0);
	cone.q_radius = 0.8f;
	std::shared_ptr<rt::ConeSceneElement> element(new rt::ConeSceneElement(cone, meshMat));
	_scene = _scene->addElement(element);

	_pt = std::shared_ptr<rt::PathTracer>(new rt::PathTracer(_scene));
}

//--------------------------------------------------------------
void ofApp::update(){
	_pt->step();
	
	if (ofGetFrameNum() % 30 == 0) {
		_image.setFromPixels(toOf(_pt->_image));
	}
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

	if (100 < _sw->elapsed()) {
		static bool write = false;
		if (write == false) {
			write = true;

			char name[] = "";
			sprintf(name, "image_100s.png");
			_image.save(name);
		}
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 's') {
		char name[] = "";
		sprintf(name, "image.png");
		_image.save(name);
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
