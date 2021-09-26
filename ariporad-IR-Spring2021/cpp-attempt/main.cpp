#include "opencv2/opencv.hpp"
#include "process_frame.cpp"
#include <iostream>

using namespace std;
using namespace cv;

int main( int argc, char** argv )
{
    VideoCapture cap(0); //capture the video from webcam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

	cout << "Opened Webcam!" << endl;

	while (true) {
		Mat frame;

		cap >> frame; // capture a frame

		// Not sure what the implications of this are
		if (frame.empty()) break;

		imshow("Frame", frame);

		process_frame(frame);

		break;

		// // Wait for 1ms, but the loop is also rate-limited by the camera's framerate.
		// char c = waitKey(1); 
		// if (c == 27) break; // Exit if ESC is pressed
	}

	cap.release();

	destroyAllWindows();

	return 0;
}