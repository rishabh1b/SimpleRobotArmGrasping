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
  /**
   * @brief      VisionManager Constructor
   *
   * @param[in]  length   The length of the table
   * @param[in]  breadth  The breadth of the table
   */
	VisionManager(float length, float breadth);
	/**
	 * @brief      Gets the 2d location of object in camera frame
	 *
	 * @param[in]  img   The image
	 * @param      x     x postion of the object
	 * @param      y     y position of the object
	 */
	void get2DLocation(cv::Mat img, float& x, float& y);

 private:
 	/**
 	 * @brief      detect2DObject processes the image to isolate object
 	 *
 	 * @param      pixel_x  postion of the object in x-pixels
 	 * @param      pixel_y  positino of the object in y-pixels
 	 */
	void detect2DObject(float& pixel_x, float& pixel_y);
	/**
	 * @brief      convertToMM converts pixel measurement to metric
	 *
	 * @param      pixel_mm_x  The pixel millimeters per x
	 * @param      pixel_mm_y  The pixel millimeters per y
	 */
	void convertToMM(float& pixel_mm_x, float& pixel_mm_y);
	/**
	 * @brief      detectTable isolates the table to get pixel to metric conversion
	 */
	void detectTable();
	/**
	 * @brief pixels per mm in x for the camera
	 */
	float pixels_permm_x;
	/**
	 * @brief pixels per mm in y for the camera
	 */
	float pixels_permm_y;
	/**
	 * @brief curr_pixel_centre_x is the object location in x
	 */
	float curr_pixel_centre_x;
	/**
	 * @brief curr_pixel_centre_y is the object location in y
	 */
	float curr_pixel_centre_y;
	/**
	 * @brief table length in meters
	 */
	float table_length;
	/**
	 * @brief table breadth in meters
	 */
	float table_breadth;
	/**
	 * @brief centre of the image in pixels x
	 */
	float img_centre_x_;
	/**
	 * @brief centre of the image in pixels y
	 */
	float img_centre_y_;
	/**
	 * @brief boolean to prevent detectTable running every time
	 */
	bool pixel_size_identified;
	/**
	 * @brief curr_img is the image currently being processed
	 */
	cv::Mat curr_img;
};
