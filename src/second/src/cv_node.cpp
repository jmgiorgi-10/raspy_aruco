//MARKER CREATION

#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;

int main() {

	cout << "hey bro whatsup man" << "\n";

	Mat image;
	Mat markerImage;
	Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	aruco::drawMarker(dictionary, 23, 2000, markerImage, 1);

	imshow("here it is man", markerImage);
	waitKey(0);

	return 0;

}