#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

class VisionManager {
  public :
	VisionManager(float length, float breadth);
	void get2DLocation(cv::Mat img, float& x, float& y);

  private:
	void detect2DObject(float& pixel_x, float& pixel_y);
	void convertToMM(float& pixel_mm_x, float& pixel_mm_y);
	void detectTable();
	float pixels_permm_x;
	float pixels_permm_y;
	float curr_pixel_centre_x;
	float curr_pixel_centre_y;
	float table_length;
	float table_breadth;
	float img_centre_x_;
	float img_centre_y_;
	bool pixel_size_identified;
	cv::Mat curr_img;
};