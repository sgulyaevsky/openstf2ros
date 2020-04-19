#include "minicap.h"
#include <sstream>

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include "opencv2/imgcodecs.hpp"

#include <cv_bridge/cv_bridge.h>

Minicap::Minicap(
        ros::NodeHandle &node_handle,
        int width, int height,
        int downscale_factor,
        const std::string &adb_path, const std::string &dev_dir,
        int port) :
        m_width(width),
        m_height(height),
        m_downscale_factor(downscale_factor),
        m_adb_path(adb_path),
        m_dev_dir(dev_dir),
        m_port(port),
        m_image_pub(node_handle.advertise<sensor_msgs::Image>("openstf_screen", 10, true)),
        m_background_thread(&Minicap::run, this) {

}

Minicap::~Minicap() {
    //TODO: make thread-safe
    if (m_adb_minicap)
        m_adb_minicap->kill();

    m_background_thread.join();
}

void Minicap::run() {
    if (m_downscale_factor < 1) {
        ROS_WARN_STREAM("downscale_factor = " << m_downscale_factor << " < 1, setting downscale_factor to 1");
        m_downscale_factor = 1;
    }

    std::stringstream minicap_args;
    minicap_args << "LD_LIBRARY_PATH=" << m_dev_dir << " " << m_dev_dir << "/minicap ";
    minicap_args << "-P " << m_height << "x" << m_height << "@" << m_height / m_downscale_factor << "x"
                 << m_height / m_downscale_factor << "/0 ";
    minicap_args << "-S -Q 100";

    ROS_INFO_STREAM("Minicap args: " << minicap_args.str());

    m_adb_minicap = std::unique_ptr<TinyProcessLib::Process>(new TinyProcessLib::Process(
            std::vector<std::string>{m_adb_path, "shell", minicap_args.str()}, "", [&](const char *bytes, size_t n) {
                ROS_INFO_STREAM(std::string(bytes, n));
            }));

    // reading loop
    read();

    auto exit_status = m_adb_minicap->get_exit_status();
    ROS_INFO_STREAM("Minicap exit code: " << exit_status);
}

void Minicap::read() {
    try {
        boost::system::error_code ec;
        boost::asio::io_service io_service;

        boost::asio::ip::tcp::socket minicap_socket(io_service);

        for (int i = 0; i < 10; i++) {
            ROS_INFO_STREAM("Connecting to minicap..");
            // wait for startup
            sleep(2);
            minicap_socket.connect(
                    boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), m_port),
                    ec);
            if (ec) {
                ROS_ERROR_STREAM(ec.message());
            } else {
                break;
            }
        }
        if (ec) {
            ROS_ERROR_STREAM(ec.message());
            return;
        }

        std::vector<char> buffer(8 * 1024 * 1024);

        // read banner
        boost::asio::read(minicap_socket, boost::asio::buffer(buffer), boost::asio::transfer_exactly(24), ec);
        if (ec) {
            ROS_ERROR_STREAM(ec.message());
            return;
        }

        ROS_INFO_STREAM(int(*reinterpret_cast<uint8_t *>(&buffer.data()[1])));
        ROS_INFO_STREAM("Real display width in pixels (minicap): " << *reinterpret_cast<uint32_t *>(&buffer.data()[6]));
        ROS_INFO_STREAM(
                "Real display height in pixels (minicap): " << *reinterpret_cast<uint32_t *>(&buffer.data()[10]));
        ROS_INFO_STREAM(
                "Virtual display width in pixels (minicap): " << *reinterpret_cast<uint32_t *>(&buffer.data()[14]));
        ROS_INFO_STREAM(
                "Virtual display height in pixels (minicap): " << *reinterpret_cast<uint32_t *>(&buffer.data()[18]));

        // read images
        cv::Mat img;
        for (;;) {
            // read frame size
            boost::asio::read(minicap_socket, boost::asio::buffer(buffer), boost::asio::transfer_exactly(4), ec);
            if (ec) {
                ROS_ERROR_STREAM(ec.message());
                return;
            }

            // read frame
            uint32_t frame_size = *reinterpret_cast<uint32_t *>(&buffer[0]);
            buffer.resize(frame_size);
            boost::asio::read(minicap_socket, boost::asio::buffer(buffer), boost::asio::transfer_exactly(frame_size),
                              ec);
            if (ec) {
                ROS_ERROR_STREAM(ec.message());
                return;
            }

            // decode and publish
            cv::imdecode(cv::Mat(buffer), cv::IMREAD_COLOR, &img);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
            m_image_pub.publish(msg);
        }
    }
    catch (std::exception &e) {
        ROS_ERROR_STREAM(e.what());
    }
    catch (...) {
        ROS_ERROR_STREAM("Unknown exception.");
    }
}
