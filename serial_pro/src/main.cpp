//
// Created by lbw on 25-1-22.
//
#include "Judger.h"

int main(int argc,char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Judger>());
    rclcpp::shutdown();
}