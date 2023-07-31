#ifndef TESSERACT_WRAPPER_H
#define TESSERACT_WRAPPER_H

// ROS2 imports
#include "rclcpp/rclcpp.hpp"
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/msg/string.hpp"

// CV2 imports
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// OCR imports
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

// utils
#include <bits/stdc++.h>
#include <boost/algorithm/string.hpp>

class TesseractWrapper : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
    tesseract::TessBaseAPI *api;
    std_msgs::msg::String ocr_msg;
    bool debug_{true};

public:
    TesseractWrapper();
    ~TesseractWrapper();
    void image_cb(const sensor_msgs::msg::Image::SharedPtr msg);
    void binarize_img(cv::Mat orig_img, cv::Mat *binarized_img);
    void apply_ocr();
};

#endif // TESSERACT_WRAPPER_H