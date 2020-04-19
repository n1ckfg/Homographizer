#pragma once

#include "ofMain.h"
#include "ofxCv.h"

class ofApp : public ofBaseApp {

	public:
		void setup();
		void update();
		void draw();
		
		bool movePoint(vector<ofVec2f>& points, ofVec2f point);
		void drawPoints(vector<ofVec2f>& points);
	
		ofImage left, right, warpedColor;
		vector<ofVec2f> leftPoints, rightPoints;
		bool movingPoint;
		ofVec2f* curPoint;
		bool saveMatrix;
		bool homographyReady;
	
		cv::Mat left_mat, right_mat, left_undistort, right_undistort, homography;
		ofxCv::Calibration homographyCalibration, distortionCalibration_L, distortionCalibration_R;
		bool useUndistort = false;

        int counter = 0;
        string inputFileType = "png";
        string outputFileType = "png";
        bool finished = false;
    
};
