#include "tesseract-ocr-ros2/tesseract_ocr_node.hpp"

TesseractWrapper::TesseractWrapper() : Node("drone_coords")
{
    pub = this->create_publisher<std_msgs::msg::String>("/ocr_text", 1);
    img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 1, std::bind(&TesseractWrapper::image_cb, this, std::placeholders::_1));

    api = new tesseract::TessBaseAPI();
    if (api->Init(NULL, "eng"))
    {
        RCLCPP_ERROR(this->get_logger(), "Could not initialize tesseract.\n");
    }
    api->SetPageSegMode(tesseract::PSM_AUTO);
    api->SetVariable("debug_file", "/dev/null");
    if (debug_)
        cv::namedWindow("Binarized Window");
}

TesseractWrapper::~TesseractWrapper()
{
    if (debug_)
        cv::destroyWindow("Binarized Window");
}

void TesseractWrapper::binarize_img(cv::Mat orig_img, cv::Mat *binarized_img)
{
    cv::Mat greyMat;
    cv::cvtColor(orig_img, greyMat, cv::COLOR_BGR2GRAY);
    cv::threshold(greyMat, *binarized_img, 100, 255, cv::THRESH_BINARY);
    api->SetImage(binarized_img->data, binarized_img->cols, binarized_img->rows, 1, binarized_img->step);
}

void TesseractWrapper::apply_ocr()
{
    char *outText = api->GetUTF8Text();
    ocr_msg.data = outText;
    if (debug_)
        RCLCPP_INFO(this->get_logger(), "Data retrieved from image: %s", outText);
}

void TesseractWrapper::image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat binary_image;
    TesseractWrapper::binarize_img(cv_ptr->image, &binary_image);
    TesseractWrapper::apply_ocr();
    if (debug_)
    {
        // Update GUI Window
        cv::imshow("Binarized Window", binary_image);
        cv::waitKey(3);
    }

    pub->publish(ocr_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TesseractWrapper>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}