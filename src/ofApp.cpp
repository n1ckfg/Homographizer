#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {
	ofSetVerticalSync(true);

	FileStorage settings(ofToDataPath("calibration/target/target_settings.yml"), FileStorage::READ);
	if (settings.isOpened()) {
		int xCount = settings["xCount"], yCount = settings["yCount"];
		homographyCalibration.setPatternSize(xCount, yCount);
		distortionCalibration_L.setPatternSize(xCount, yCount);
		distortionCalibration_R.setPatternSize(xCount, yCount);

		float squareSize = settings["squareSize"];
		homographyCalibration.setSquareSize(squareSize);
		distortionCalibration_L.setSquareSize(squareSize);
		distortionCalibration_R.setSquareSize(squareSize);

		CalibrationPattern patternType;
		switch (settings["patternType"]) {
			case 0: patternType = CHESSBOARD; break;
			case 1: patternType = CIRCLES_GRID; break;
			case 2: patternType = ASYMMETRIC_CIRCLES_GRID; break;
		}
		homographyCalibration.setPatternType(CHESSBOARD); // patternType);
		distortionCalibration_L.setPatternType(CHESSBOARD); // patternType);
		distortionCalibration_R.setPatternType(CHESSBOARD); // patternType);
	}

    ofDirectory leftCalibrationDir("calibration/left/");
    leftCalibrationDir.allowExt(inputFileType);
    leftCalibrationDir.listDir();
    leftCalibrationDir.sort();
	int leftCalibrationDirCount = leftCalibrationDir.size();

	ofDirectory rightCalibrationDir("calibration/right/");
	rightCalibrationDir.allowExt(inputFileType);
    rightCalibrationDir.listDir();
    rightCalibrationDir.sort();
    int rightCalibrationDirCount = rightCalibrationDir.size();

	cout << "calib L: " << leftCalibrationDirCount << ", calib R: " << rightCalibrationDirCount << endl;

	// 1. Calculate lens distortion
	string distortionCalibrationUrl_L = "calibration/distortion_L.yml";
	ofFile previousDistortion_L(distortionCalibrationUrl_L);
	distortionCalibration_L.setFillFrame(true); // true by default

	string distortionCalibrationUrl_R = "calibration/distortion_R.yml";
	ofFile previousDistortion_R(distortionCalibrationUrl_R);
	distortionCalibration_R.setFillFrame(true); // true by default

	if (previousDistortion_L.exists() && previousDistortion_R.exists()) {
		// 1.1. Load the previous distortion if it's available
		cout << "Found existing distortion calibration files." << endl;
		distortionCalibration_L.load(distortionCalibrationUrl_L);
		distortionCalibration_R.load(distortionCalibrationUrl_R);
	} else {
		// 1.2. Otherwise calculate the distortion.
		for (int i = 0; i < leftCalibrationDir.size(); i++) {
			string leftCalibrationUrl = leftCalibrationDir.getPath(i);
			cout << "calib L " << (i + 1) << "/" << leftCalibrationDirCount << ": " << leftCalibrationUrl << endl;
			left.load(leftCalibrationUrl);
			distortionCalibration_L.add(toCv(left));

			string rightCalibrationUrl = rightCalibrationDir.getPath(i);
			cout << "calib R " << (i + 1) << "/" << rightCalibrationDirCount << ": " << rightCalibrationUrl << endl;
			right.load(rightCalibrationUrl);
			distortionCalibration_R.add(toCv(right));
		}

		distortionCalibration_L.calibrate();
		distortionCalibration_L.clean();
		distortionCalibration_L.save(distortionCalibrationUrl_L);

		distortionCalibration_R.calibrate();
		distortionCalibration_R.clean();
		distortionCalibration_R.save(distortionCalibrationUrl_R);
	}

	// 2. Calculate the homography.
    string homographyCalibrationUrl = "calibration/homography.yml";
    ofFile previousHomography(homographyCalibrationUrl);
    
	if (previousHomography.exists()) {
		// 2.1. load the previous homography if it's available
		cout << "Found existing homography calibration file." << endl;
        FileStorage fs(ofToDataPath(homographyCalibrationUrl), FileStorage::READ);
        fs["homography"] >> homography;
        homographyReady = true;
    } else {
		// 2.2. Otherwise calculate the homography.
        for (int i=0; i<leftCalibrationDir.size(); i++) {
            string leftCalibrationUrl = leftCalibrationDir.getPath(i);
			cout << "calibH L " << (i + 1) << "/" << leftCalibrationDirCount << ": " << leftCalibrationUrl << endl;
			left.load(leftCalibrationUrl);
			left_mat = toCv(left);

			string rightCalibrationUrl = rightCalibrationDir.getPath(i);
            cout << "calibH R " << (i+1) << "/" << rightCalibrationDirCount << ": " << rightCalibrationUrl << endl;
			right.load(rightCalibrationUrl);
			right_mat = toCv(right);
			
			vector<Point2f> leftBoardPoints;
			vector<Point2f> rightBoardPoints;

			if (useUndistort) {
				// apply undistortion
				imitate(left_undistort, left_mat);
				distortionCalibration_L.undistort(left_mat, left_undistort);
				homographyCalibration.findBoard(left_undistort, leftBoardPoints);

				imitate(right_undistort, right_mat);
				distortionCalibration_R.undistort(right_mat, right_undistort);
				homographyCalibration.findBoard(right_undistort, rightBoardPoints);
			} else {
				homographyCalibration.findBoard(left_mat, leftBoardPoints);
				homographyCalibration.findBoard(right_mat, rightBoardPoints);
			}

            for (int i = 0; i < leftBoardPoints.size(); i++) {
                Point2f pt = leftBoardPoints[i];
                leftPoints.push_back(ofVec2f(pt.x, pt.y));
            }

            for (int i = 0; i < rightBoardPoints.size(); i++) {
                Point2f pt = rightBoardPoints[i];
                rightPoints.push_back(ofVec2f(pt.x + right.getWidth(), pt.y));
            }
        }
        
        //movingPoint = false;
        //saveMatrix = false;
        homographyReady = false;

		// 3. Finally, apply the homography.
		if (leftPoints.size() >= 4) {
			vector<Point2f> srcPoints, dstPoints;
			for (int i = 0; i < leftPoints.size(); i++) {
				srcPoints.push_back(Point2f(rightPoints[i].x - left.getWidth(), rightPoints[i].y));
				dstPoints.push_back(Point2f(leftPoints[i].x, leftPoints[i].y));
			}

			// generate a homography from the two sets of points
			homography = findHomography(Mat(srcPoints), Mat(dstPoints));

			//if(saveMatrix) {
			FileStorage fs(ofToDataPath("calibration/homography.yml"), FileStorage::WRITE);
			fs << "homography" << homography;
			//saveMatrix = false;
			//}
			homographyReady = true;
		} else {
			cout << "ERROR did not find at least four correspondences." << endl;
		}
    }
}

void ofApp::update() {
    if (finished) {
        warpedColor.update();
    } else {      
        if (homographyReady) {
			ofDirectory leftDir("input_left");
			leftDir.allowExt(inputFileType);
			leftDir.listDir();
			leftDir.sort();
			int leftDirCount = leftDir.size();

            ofDirectory rightDir("input_right");
            rightDir.allowExt(inputFileType);
            rightDir.listDir();
            rightDir.sort();
            int rightDirCount = rightDir.size();
            
            string leftUrl = leftDir.getPath(counter);
            cout << "main L " << (counter + 1) << "/" << leftDirCount << ": " << leftUrl << endl;
            left.load(leftUrl);
			
			if (useUndistort) {
				left_mat = toCv(left);
				imitate(left_undistort, left_mat);
				distortionCalibration_L.undistort(left_mat, left_undistort);
				toOf(left_undistort, left);
			}

			string outputUrl_L = "output_left/output_L_" + ofToString(counter) + "." + outputFileType;
			left.save(outputUrl_L);

			string rightUrl = rightDir.getPath(counter);
			cout << "main R " << (counter + 1) << "/" << rightDirCount << ": " << rightUrl << endl;
			right.load(rightUrl);
			
			if (useUndistort) {
				right_mat = toCv(right);
				imitate(right_undistort, right_mat);
				distortionCalibration_R.undistort(right_mat, right_undistort);
				toOf(right_undistort, right);
			}

            // this is how you warp one ofImage into another ofImage given the homography matrix
            // CV INTER NN is 113 fps, CV_INTER_LINEAR is 93 fps
			imitate(warpedColor, right);
			warpPerspective(right, warpedColor, homography, CV_INTER_LINEAR);
            warpedColor.update();
            string outputUrl_R = "output_right/output_R_" + ofToString(counter) + "." + outputFileType;
            warpedColor.save(outputUrl_R);
            
			if (counter < rightDirCount-1) {
                counter++;
            } else {
                finished = true;
            }
        }
    }
}

void ofApp::drawPoints(vector<ofVec2f>& points) {
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

