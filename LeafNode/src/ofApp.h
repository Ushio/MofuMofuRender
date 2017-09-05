#pragma once

#include "ofMain.h"
#include "ofxImGui.h"

#include "scene.hpp"
#include "conelbox.hpp"
#include "stopwatch.hpp"
#include "bezier.hpp"
#include "stats.hpp"
#include "aabb.hpp"
#include "obb.hpp"
#include "pca.hpp"

struct BeizerFur {
	double radius = 0.0;
	rt::BezierQuadratic3D bezier;
};
struct BezierLeaf {
	std::vector<BeizerFur> beizers;
};

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofEasyCam _camera;
	ofxImGui::Gui _imgui;
	ofImage _image;

	std::shared_ptr<rt::Scene> _scene;
	rt::BezierQuadratic3D _bezierQuadratic;

	
	int _index = 0;
	std::vector<BezierLeaf> _leafs;

	float _ray_y = 0.0f;
	float _ray_z = 0.0f;

	bool _hasNode = false;
	rt::BVHNode _node;
	std::vector<rt::BezierEntity> _beziers;

	int _viewDepth = 0;
};
