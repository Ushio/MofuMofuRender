#pragma once

#include "ofMain.h"
#include "ofxImGui.h"
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
		
	ofxImGui::Gui _imgui;
	ofEasyCam  _camera;

	bool _check = false;
	float _v = 0.5;
	float _theta = 0;
	float _phi = 0;
};
