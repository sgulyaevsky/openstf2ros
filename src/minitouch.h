#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "process.hpp"

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <thread>
#include <memory>

class Minitouch {
public:
    Minitouch(Minitouch const &) = delete;

    Minitouch &operator=(Minitouch const &) = delete;

    Minitouch(ros::NodeHandle &node_handle, const std::string &adb_path, const std::string &dev_dir, int port);

    ~Minitouch();

private:
    void run();

    void read();

    void callback(const std_msgs::String::ConstPtr &msg);

    std::string m_adb_path;
    std::string m_dev_dir;
    int m_port;
    ros::Subscriber m_command_sub;

    std::unique_ptr<TinyProcessLib::Process> m_adb_minitouch;

    boost::asio::io_service m_io_service;
    boost::asio::ip::tcp::socket m_minitouch_socket;

    std::thread m_background_thread;
};
