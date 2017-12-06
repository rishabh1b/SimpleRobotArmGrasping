#include "robotarm_grasping/VisionManager.h"

VisionManager::VisionManager(float length, float breadth) {
	this->table_length = length;
	this->table_breadth = breadth;
	pixel_size_identified = false;
}

void VisionManager::get2DLocation(cv::Mat img, float&x, float& y) {
this->curr_img = img;
if (!pixel_size_identified) {
	detectTable();
}
detect2DObject(x, y);
convertToMM(x,y);
}

void VisionManager::detectTable() {
	// Extract Table from the image and assign values to pixel_per_mm fields
}

void VisionManager::detect2DObject(float& pixel_x, float& pixel_y) {
	// Implement Color Thresholding and contour findings to get the location of object to be grasped in 2D
}

void VisionManager::convertToMM(float& x, float& y) {
	// Convert from pixel to world co-ordinates in the camera frame
}

// Temporary Main Function for testing- This should go away later
int main(int argc, char** argv ) {
	if ( argc != 2 )
    {
        printf("usage: VisionManager <Image_Path>\n");
        return -1;
    }

    cv::Mat image;
    image = cv::imread( argv[1], 1 );

    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }

    float length = 1;
    float breadth = 0.6;
    float obj_x, obj_y;

    VisionManager vm(length, breadth);
    vm.get2DLocation(image, obj_x, obj_y);

    std::cout<< " X-Co-ordinate in Camera Frame" << obj_x;
    std::cout<< " Y-Co-ordinate in Camera Frame" << obj_y;
}




