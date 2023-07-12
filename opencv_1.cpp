#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using std::cerr; using std::endl;


int main(int argc, char **argv)
{
	cv::VideoCapture cap;
	
	const char* gst = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, width=1280, height=720, format=(string)BGRx ! videoconvert ! appsink";

	cap.open(gst);
	

	if (!cap.isOpened())
	{
		cerr << "Fallo en abrir la camara." << endl;
		return -1;
	}


	cv::Mat frame;
	while (true)
	{
		cap.read(frame);
		cv::imshow("Camara", frame);

		if (cv::waitKey(30) >= 0) break;
	}

	cap.release();
	return 0;
}

