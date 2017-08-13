#include "ofMain.h"

#include "ofApp.h"
#include "ofAppNoWindow.h"

//========================================================================
int main( ){
#if NO_WINDOW
	ofAppNoWindow window;
	ofSetupOpenGL(&window, 0, 0, OF_WINDOW);
#else
	// 
	ofSetupOpenGL(1024, 768, OF_WINDOW);			// <-------- setup the GL context
#endif

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new ofApp());

}
