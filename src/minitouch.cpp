#include "minitouch.h"
#include <sstream>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <memory>

Minitouch::Minitouch(ros::NodeHandle &node_handle, const std::string &adb_path, const std::string &dev_dir, int port) :
        m_adb_path(adb_path),
        m_dev_dir(dev_dir),
        m_port(port),
        m_command_sub(node_handle.subscribe("openstf_control_messages", 1000, &Minitouch::callback, this)),
        m_minitouch_socket(m_io_service),
        m_background_thread(&Minitouch::run, this) {
}

Minitouch::~Minitouch() {
    //TODO: make thread-safe
    if (m_adb_minitouch)
        m_adb_minitouch->kill();

    m_background_thread.join();
}


void Minitouch::callback(const std_msgs::String::ConstPtr &msg) {
    boost::system::error_code ec;

    ROS_DEBUG_STREAM(msg->data);
    boost::asio::write(m_minitouch_socket, boost::asio::buffer(msg->data),
                       boost::asio::transfer_all(), ec);
    if (ec)
        ROS_ERROR_STREAM(ec.message());
}

void Minitouch::run() {
    m_adb_minitouch = std::unique_ptr<TinyProcessLib::Process>(new TinyProcessLib::Process(
            std::vector<std::string>{m_adb_path, "shell", m_dev_dir + "/minitouch"}, "",
            [&](const char *bytes, size_t n) {
                ROS_INFO_STREAM(std::string(bytes, n));
            }));

    // reading loop
    read();

    auto exit_status = m_adb_minitouch->get_exit_status();
    ROS_INFO_STREAM("Minicap exit code: " << exit_status);
}

void Minitouch::read() {
    try {
        boost::system::error_code ec;
        for (int i = 0; i < 10; i++) {
            ROS_INFO_STREAM("Connecting to minitouch..");
            // wait for startup
            sleep(2);
            m_minitouch_socket.connect(
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

        std::vector<char> buffer(8 * 1024);
        for (;;) {
            size_t len = m_minitouch_socket.read_some(boost::asio::buffer(buffer), ec);
            ROS_INFO_STREAM(std::string(buffer.begin(), buffer.begin() + len));

            if (ec) {
                ROS_ERROR_STREAM(ec.message());
                return;
            }
        }
    }
    catch (std::exception &e) {
        ROS_ERROR_STREAM(e.what());
    }
    catch (...) {
        ROS_ERROR_STREAM("Unknown exception.");
    }
}
