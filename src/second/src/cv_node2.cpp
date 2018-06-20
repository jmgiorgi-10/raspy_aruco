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
	aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);

	aruco::drawDetectedMarkers(inputImage, markerCorners, markerIds);
	imshow("here it is bruu", inputImage);
	waitKey(0);

	return(0);

}