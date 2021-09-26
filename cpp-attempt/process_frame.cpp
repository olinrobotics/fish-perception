#include "opencv2/opencv.hpp"
#include <cstdint>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>

using namespace std;
using namespace cv;


Mat process_frame(Mat frame)
{
	// TODO: these both need calibration (currently total guesses)
	const int LASER_ZERO_PIXEL_ROW = frame.rows / 2;
	const double METERS_OFFSET_PER_PIXEL_ROW = 0.001;

	Mat pos(Size(1, frame.cols), CV_16U);

	for (int i = frame.cols / 2; i < frame.cols; i++)
	{
		Mat col = frame.col(i);
		Mat hsv, masked;

		cvtColor(col, hsv, COLOR_BGR2HSV);
		inRange(hsv, Scalar(36, 25, 25), Scalar(70, 255, 255), masked);

		bool on_streak = false;
		int max_streak_idx = 0;
		int max_streak_len = 0;
		int cur_streak_idx = 0;
		int cur_streak_len = 0;

		for (int i = 0; i < masked.rows; i++) {
			if (masked.at<ushort>(i) > 0) {
				if (!on_streak) {
					on_streak = true;
					cur_streak_idx = i;
				}
				cur_streak_len++;

				if (cur_streak_len > max_streak_len) {
					max_streak_idx = cur_streak_idx;
					max_streak_len = cur_streak_len;
				}
			} else {
				on_streak = false;
				cur_streak_idx = 0;
				cur_streak_len = 0;
			}
		}

        pos.at<uint16_t>(frame.cols) = max_streak_idx + (max_streak_len / 2);
    }


	Mat adjusted_pos = pos - LASER_ZERO_PIXEL_ROW;
	Mat calculated_distances = adjusted_pos * METERS_OFFSET_PER_PIXEL_ROW;

	return calculated_distances;
}