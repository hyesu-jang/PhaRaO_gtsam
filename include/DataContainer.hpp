#pragma once

#include <vector>
#include <array>

#define NUM 5

struct DataContainer
{
    bool initialized;

    std::vector<cv::Mat> window_list;
    std::vector<cv::Mat> window_list_cart;
    std::vector<cv::Mat> window_list_cart_f;

    std::vector<cv::Mat> keyf_list;
    std::vector<cv::Mat> keyf_list_cart;
    std::vector<cv::Mat> keyf_list_cart_f;

    std::vector<ros::Time> stamp_list;
    std::vector<ros::Time> keyf_stamp_list;

    std::array<std::array<double, 3>, NUM> del_list;
    std::array<std::array<double, 3>, NUM> odom_list;
};