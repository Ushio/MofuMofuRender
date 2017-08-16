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

	cameraSetting._focalLength = 1.2;
	cameraSetting._lensRadius = 0.015;

	// 引き
	//cameraSetting._eye = Vec3(2.507, 1.297, 1.648);
	//cameraSetting._lookat = Vec3(1.81928, 0.8427957, 1.081661);
	//cameraSetting._up = Vec3(-0.4395704, 0.8813744, -0.1730812);
	//cameraSetting._fov = 1.047198;

	std::vector<std::shared_ptr<rt::SceneElement>> sceneElements;

	{
		std::vector<Vec3> vertices = {
			rt::Vec3(-0.5, 0.5, 0.0),
			rt::Vec3( 0.5, 0.5, 0.0),
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

	// キーライト
	{
		std::vector<Vec3> vertices = {
			rt::Vec3(-0.5, 0.5, 0.0),
			rt::Vec3( 0.5, 0.5, 0.0),
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
			LambertianMaterial(rt::Vec3(1.0, 0.9, 0.8) * 10.0, rt::Vec3(0.75)),
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

	//{
	//	std::vector<rt::BezierEntity> beziers;
	//	rt::Xor random;
	//	for (int i = 0; i < 300; ++i) {
	//		auto p = rt::uniform_in_unit_circle(&random);
	//		auto p1 = rt::Vec3(p.x, -1.0, p.y);
	//		auto cp = p1 + rt::Vec3(random.uniform(-0.1, 0.1), 0.2, random.uniform(-0.1, 0.1));
	//		auto p2 = cp + rt::Vec3(random.uniform(-0.1, 0.1), 0.2, random.uniform(-0.1, 0.1));

	//		rt::BezierQuadratic3D bz(p1, cp, p2);

	//		rt::BezierEntity e;
	//		e.bezier = bz;
	//		e.sigma_a = rt::Vec3(random.uniform(0.0, 0.8), random.uniform(0.0, 0.8), random.uniform(0.0, 0.8));
	//		e.radius = random.uniform(0.003, 0.008);
	//		beziers.push_back(e);
	//	}
	//	std::shared_ptr<rt::BezierBVHSceneElement> element(new rt::BezierBVHSceneElement(beziers));
	//	_scene = _scene->addElement(element);
	//}

	_scene = scene_fromUnity();

#if 1
	{
		std::vector<rt::TriangleFace> triangles = rt::loadObjWithTangent(ofToDataPath("araiguma/basic.obj"));
		std::vector<double> areas;
		for (int i = 0; i < triangles.size(); ++i) {
			double area = rt::triangle_area(triangles[i].v[0], triangles[i].v[1], triangles[i].v[2]);
			areas.push_back(area);
		}
		rt::AreaUniformSampler areaUniformSampler(areas);

		std::vector<rt::BezierEntity> beziers;

		ofPixels lengthMap;
		ofPixels patternMap;
		ofLoadImage(lengthMap, "araiguma/Length.png");
		ofLoadImage(patternMap, "araiguma/Pattern.png");

		rt::Xor random;
		for (int i = 0; i < 30000; ++i) {
			auto s = areaUniformSampler.sample(&random);
			auto tri = triangles[s];
			auto sample = rt::uniform_on_triangle(random.uniform(), random.uniform());
			auto p = sample.evaluate(tri.v[0], tri.v[1], tri.v[2]);

			auto n = sample.evaluate(tri.n[0], tri.n[1], tri.n[2]);
			n = glm::normalize(n);

			// 
			auto texcoord = sample.evaluate(tri.t[0], tri.t[1], tri.t[2]);
			int ux = lengthMap.getWidth() * texcoord.x;
			int uv = lengthMap.getHeight() * (1.0 - texcoord.y);
			ux = glm::clamp(ux, 0, (int)lengthMap.getWidth() - 1);
			uv = glm::clamp(uv, 0, (int)lengthMap.getHeight() - 1);
			auto hairLength = (double)lengthMap.getColor(ux, uv).r / 255.0;
			auto hairPattern = (double)patternMap.getColor(ux, uv).r / 255.0;

			auto tangent = sample.evaluate(tri.tangent[0], tri.tangent[1], tri.tangent[2]);
			auto bitangent = sample.evaluate(tri.bitangent[0], tri.bitangent[1], tri.bitangent[2]);
			tangent = glm::normalize(tangent);
			bitangent = glm::normalize(bitangent);

			// 毛のカーブ
			auto n0 = glm::rotate(n, glm::radians(25.0), tangent);
			auto n1 = glm::rotate(n, glm::radians(60.0 + random.uniform(-20.0, 10.0)), tangent);

			auto random_1 = rt::Vec3(random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)) * 0.02;
			auto random_2 = rt::Vec3(random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)) * 0.04;

			// 毛の長さ
			double baseLength = random.uniform(0.0, 0.05) + random.uniform(0.0, 0.05) + random.uniform(0.0, 0.05);

			rt::BezierQuadratic3D bz(p,
				p + n0 * baseLength * hairLength + random_1 ,
				p + n0 * baseLength * hairLength + n1 * baseLength * hairLength + random_2 + bitangent * random.uniform(-1.0, 1.0) * 0.006
			);

			// 
			using namespace rt;
			Transform transform = Transform(Vec3(0.943, -0.11, 0.64), glm::angleAxis(-4.125526, Vec3(-0.7148883, 0.583186, -0.3857836)), Vec3(0.716475, 0.7164751, 0.7164751));
			Quat base = glm::angleAxis(glm::radians(180.0), Vec3(0, 1, 0));
			for (int j = 0; j < 3; ++j) {
				bz[j] = transform * (base * bz[j]);
			}

			// 若干色にノイズを加えたほうが自然になる
			double noisescale = 8.0;
			double noise = glm::mix(1.0, 2.0, (double)ofNoise(p.x * noisescale, p.y * noisescale, p.z * noisescale));
			rt::BezierEntity e;
			e.bezier = bz;
			e.sigma_a = glm::mix(rt::Vec3(0.4, 0.6, 1.5) * 5.0, rt::Vec3(0.45, 0.65, 0.8) * noise * 0.4, hairPattern);
			e.radius = 0.001;
			beziers.push_back(e);
		}
		std::shared_ptr<rt::BezierBVHSceneElement> element(new rt::BezierBVHSceneElement(beziers));
		_scene = _scene->addElement(element);
	}
#endif

	_pt = std::shared_ptr<rt::PathTracer>(new rt::PathTracer(_scene));

	ofPixels ibl;
	ofLoadImage(ibl, "IBL.png");
	ibl.setImageType(OF_IMAGE_COLOR);

	rt::IBL iblImage(ibl.getWidth(), ibl.getHeight());
	const uint8_t *pixels = ibl.getPixels();
	for (int y = 0; y < iblImage.height(); ++y) {
		for (int x = 0; x < iblImage.width(); ++x) {
			int index = (y * ibl.getWidth() + x) * 3;
			*iblImage.pixel(x, y) = rt::Vec3(pixels[index], pixels[index + 1], pixels[index + 2]) / 255.0;
		}
	}
	_pt->_ibl = iblImage;

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
