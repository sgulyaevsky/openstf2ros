#pragma once

#include <ros/ros.h>
#include "process.hpp"
#include <thread>
#include <memory>

class Minicap {
public:
    Minicap(Minicap const &) = delete;

    Minicap &operator=(Minicap const &) = delete;

    Minicap(ros::NodeHandle &node_handle,
            int width, int height,
            int downscale_factor,
            const std::string &adb_path, const std::string &dev_dir,
            int port);

    ~Minicap();

private:
    void run();

    void read();

    int m_width;
    int m_height;
    int m_downscale_factor;
    std::string m_adb_path;
    std::string m_dev_dir;
    int m_port;

    ros::Publisher m_image_pub;
    std::unique_ptr<TinyProcessLib::Process> m_adb_minicap;

    std::thread m_background_thread;
};
