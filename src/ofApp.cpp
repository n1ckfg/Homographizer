#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {
	ofSetVerticalSync(true);

	FileStorage settings(ofToDataPath("settings.yml"), FileStorage::READ);
	if (settings.isOpened()) {
		int xCount = settings["xCount"], yCount = settings["yCount"];
		calibration.setPatternSize(xCount, yCount);
		float squareSize = settings["squareSize"];
		calibration.setSquareSize(squareSize);
		CalibrationPattern patternType;
		switch (settings["patternType"]) {
		case 0: patternType = CHESSBOARD; break;
		case 1: patternType = CIRCLES_GRID; break;
		case 2: patternType = ASYMMETRIC_CIRCLES_GRID; break;
		}
		calibration.setPatternType(CHESSBOARD); // patternType);
	}

    ofDirectory leftDir("left/");
    ofDirectory rightDir("right/");
    leftDir.allowExt("jpg");
    rightDir.allowExt("jpg");
    leftDir.listDir();
    leftDir.sort();
    rightDir.listDir();
    rightDir.sort();
    int leftDirCount = leftDir.size();
    int rightDirCount = rightDir.size();
    cout << "left: " << leftDirCount << ", right: " << rightDirCount << endl;
    
    // load the previous homography if it's available
    ofFile previous("homography.yml");
    if (previous.exists()) {
        cout << "Found existing calibration file." << endl;
        FileStorage fs(ofToDataPath("homography.yml"), FileStorage::READ);
        fs["homography"] >> homography;
        homographyReady = true;
        
        string rightUrl = rightDir.getPath(0);
        right.load(rightUrl);
    } else {
        for (int i=0; i<leftDir.size(); i++) {
            string leftUrl = leftDir.getPath(i);
            string rightUrl = rightDir.getPath(i);
            cout << "left " << (i+1) << "/" << leftDirCount << ": " << leftUrl << endl;
            cout << "right " << (i+1) << "/" << rightDirCount << ": " << rightUrl << endl;

            left.load(leftUrl);
            right.load(rightUrl);

            vector<Point2f> leftBoardPoints;
            vector<Point2f> rightBoardPoints;
            calibration.findBoard(toCv(left), leftBoardPoints);
            calibration.findBoard(toCv(right), rightBoardPoints);
            for (int i = 0; i < leftBoardPoints.size(); i++) {
                Point2f pt = leftBoardPoints[i];
                leftPoints.push_back(ofVec2f(pt.x, pt.y));
            }
            for (int i = 0; i < rightBoardPoints.size(); i++) {
                Point2f pt = rightBoardPoints[i];
                rightPoints.push_back(ofVec2f(pt.x + right.getWidth(), pt.y));
            }
        }

        imitate(warpedColor, right);
        
        movingPoint = false;
        saveMatrix = false;
        homographyReady = false;
    }
}

void ofApp::update() {
    if (!homographyReady) {
        if(leftPoints.size() >= 4) {
            vector<Point2f> srcPoints, dstPoints;
            for(int i = 0; i < leftPoints.size(); i++) {
                srcPoints.push_back(Point2f(rightPoints[i].x - left.getWidth(), rightPoints[i].y));
                dstPoints.push_back(Point2f(leftPoints[i].x, leftPoints[i].y));
            }
            
            // generate a homography from the two sets of points
            homography = findHomography(Mat(srcPoints), Mat(dstPoints));
            homographyReady = true;
            
            //if(saveMatrix) {
            FileStorage fs(ofToDataPath("homography.yml"), FileStorage::WRITE);
            fs << "homography" << homography;
            //saveMatrix = false;
            //}
        }
    }
	
	if(homographyReady) {
		// this is how you warp one ofImage into another ofImage given the homography matrix
		// CV INTER NN is 113 fps, CV_INTER_LINEAR is 93 fps
		warpPerspective(right, warpedColor, homography, CV_INTER_LINEAR);
		warpedColor.update();
	}
}

void drawPoints(vector<ofVec2f>& points) {
	ofNoFill();
	for(int i = 0; i < points.size(); i++) {
		ofDrawCircle(points[i], 10);
		ofDrawCircle(points[i], 1);
	}
}

void ofApp::draw() {
	
	ofSetColor(255);
	left.draw(0, 0);
	right.draw(left.getWidth(), 0);
	if(homographyReady) {
		ofEnableBlendMode(OF_BLENDMODE_ADD);
		ofSetColor(255, 128);
		warpedColor.draw(0, 0);
		ofDisableBlendMode();
	}
	
	ofSetColor(255, 0, 0);
	drawPoints(leftPoints);
	ofSetColor(0, 255, 255);
	drawPoints(rightPoints);
	ofSetColor(128);
	for(int i = 0; i < leftPoints.size(); i++) {
		ofDrawLine(leftPoints[i], rightPoints[i]);
	}
	
	ofSetColor(255);
	ofDrawBitmapString(ofToString((int) ofGetFrameRate()), 10, 20);
}

bool ofApp::movePoint(vector<ofVec2f>& points, ofVec2f point) {
	for(int i = 0; i < points.size(); i++) {
		if(points[i].distance(point) < 20) {
			movingPoint = true;
			curPoint = &points[i];
			return true;
		}
	}
	return false;
}

void ofApp::mousePressed(int x, int y, int button) {
	ofVec2f cur(x, y);
	ofVec2f rightOffset(left.getWidth(), 0);
	if(!movePoint(leftPoints, cur) && !movePoint(rightPoints, cur)) {
		if(x > left.getWidth()) {
			cur -= rightOffset;
		}
		leftPoints.push_back(cur);
		rightPoints.push_back(cur + rightOffset);
	}
}

void ofApp::mouseDragged(int x, int y, int button) {
	if(movingPoint) {
		curPoint->set(x, y);
	}
}

void ofApp::mouseReleased(int x, int y, int button) {
	movingPoint = false;
}

void ofApp::keyPressed(int key) {
	if(key == ' ') {
        //
	}
}
