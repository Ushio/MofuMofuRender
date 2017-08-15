#pragma once

#include "ofMain.h"
#include "ofxImGui.h"

#include <plog/Log.h>
#include <plog/Appenders/ColorConsoleAppender.h>

#include "scene.hpp"
#include "conelbox.hpp"
#include "path_tracing.hpp"
#include "stopwatch.hpp"
#include "stats.hpp"

#define NO_WINDOW 0

class ofApp : public ofBaseApp{
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
	ofFloatImage _radianceImage;

	std::shared_ptr<rt::Scene> _scene;
	std::shared_ptr<rt::PathTracer> _pt;
	std::shared_ptr<rt::Stopwatch> _sw;

	std::shared_ptr<rt::Stopwatch> _wholeSW;

	rt::IncrementalStatatics _stepStats;
};
