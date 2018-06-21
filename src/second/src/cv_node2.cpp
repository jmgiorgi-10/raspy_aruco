//NODE DETECTION

#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;

int main() {
	
	Mat inputImage;
	inputImage = imread("/home/joaquin/Downloads/image1.jpeg");
	vector<int> markerIds;
	vector< vector<Point2f> > markerCorners, rejectedCandidates;
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

	//cv::aruco::MarkerDetector Detector;
	//Detector.setDictionary(dictionary);


	//cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);

	aruco::drawDetectedMarkers(inputImage, markerCorners, markerIds);

	vector<Vec3d> rvecs, tvecs;
	//aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

	imshow("here it is bruu", inputImage);
	waitKey(0);
	return(0);

}