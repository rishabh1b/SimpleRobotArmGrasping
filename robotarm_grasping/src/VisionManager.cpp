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
	cv::Mat RGB[3];
	cv::Mat image = curr_img.clone();
	split(image, RGB);
	cv::Mat gray_image_red = RGB[2];
    cv::Mat gray_image_green = RGB[1];
    cv::Mat denoiseImage;
    cv::medianBlur(gray_image_red, denoiseImage, 3);

    // Threshold the Image
    cv::Mat binaryImage = denoiseImage;
    for (int i=0; i< binaryImage.rows; i++)
    { 
        for (int j=0; j< binaryImage.cols; j++)
        {
            int editValue=binaryImage.at<uchar>(i,j);
            int editValue2=gray_image_green.at<uchar>(i,j);

            if((editValue>34)&&(editValue<50) && (editValue2>18)&&(editValue2<30)) //check whether value is within range.
            {
                binaryImage.at<uchar>(i,j)=255;
            }
            else
            {
                binaryImage.at<uchar>(i,j)=0;
            }
        }
    }
    dilate(binaryImage, binaryImage, cv::Mat());

    // Get the centroid of the of the blob
    std::vector<cv::Point> nonZeroPoints;
    cv::findNonZero(binaryImage, nonZeroPoints);
    cv::Rect bbox = cv::boundingRect(nonZeroPoints);
    cv::Point pt;
    pt.x = bbox.x + bbox.width / 2;
    pt.y = bbox.y + bbox.height / 2;
    cv::circle(image, pt, 2, cv::Scalar(0,0,255), -1,8);

    // Draw Contours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours( binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    for( int i = 0; i< contours.size(); i++ )
     {
       cv::Scalar color = cv::Scalar( 255, 0, 0);
       cv::drawContours( image, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
     }

    cv::namedWindow("Centre point", cv::WINDOW_AUTOSIZE);
    cv::imshow("Centre point", image); 

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

    cv::waitKey(0);

    std::cout<< " X-Co-ordinate in Camera Frame" << obj_x;
    std::cout<< " Y-Co-ordinate in Camera Frame" << obj_y;
}




