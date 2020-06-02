#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define confThreshold 0.5
#define nmsThreshold 0.4
#define inpWidth 416
#define inpHeight 416

void DetecDnn(cv::dnn::Net& net, cv::Mat& frame, cv::Mat& blob, std::vector<std::string> classes);

// Remove the bounding boxes with low confidence using non-maxima suppression
void DetecProc(cv::Mat& frame, const std::vector<cv::Mat>& out, std::vector<std::string> classes);

// Draw the predicted bounding box
void DetecDraw(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame, std::vector<std::string> classes);

void DetecPosi();

// Get the names of the output layers
std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net);
