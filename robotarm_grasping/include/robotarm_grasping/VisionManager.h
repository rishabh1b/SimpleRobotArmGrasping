#pragma once
/**
* @file VisionManger.h Function Prototype for Vision Manager class
* @author rishabh1b(Rishabh Biyani)
* @copyright BSD 3-Clause License (c) Rishabh Biyani 2017
*/

// BSD 3-Clause License

// Copyright (c) 2017, Rishabh Biyani
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and / or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

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
