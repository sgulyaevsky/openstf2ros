#include "openstf_init.h"
#include "process.hpp"

#include <ros/ros.h>

#include <iostream>
#include <sstream>
#include <memory>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/join.hpp>

void run_command(const std::vector<std::string> &args) {
    ROS_INFO_STREAM("Run command:" << boost::algorithm::join(args, " "));
    TinyProcessLib::Process process(args, "", [&](const char *bytes, size_t n) {
        ROS_INFO_STREAM(std::string(bytes, n));
    });
    int exit_status = process.get_exit_status();
    if (exit_status != 0) {
        ROS_INFO_STREAM("Exit status: " << exit_status);
        throw std::runtime_error("run_command error");
    }
}

std::string get_command_result(const std::vector<std::string> &args) {
    ROS_INFO_STREAM("Run command:" << boost::algorithm::join(args, " "));
    std::stringstream output;
    TinyProcessLib::Process process(args, "", [&](const char *bytes, size_t n) {
        ROS_INFO_STREAM(std::string(bytes, n));
        output << std::string(bytes, n);
    });
    int exit_status = process.get_exit_status();
    if (exit_status != 0) {
        ROS_INFO_STREAM("Exit status: " << exit_status);
        throw std::runtime_error("get_command_result error");
    }

    return output.str();
}

std::vector<std::string> split(const std::string &str, const std::string &delim = "\n") {
    std::vector<std::string> strs;
    boost::split(strs, str, boost::is_any_of(delim));
    return strs;
}

std::string command_output_line(const std::vector<std::string> &args) {
    const auto &output = split(get_command_result(args));
    if (output.empty())
        return "";

    return output[0];
}

bool check_adb_server() {
    ROS_INFO_STREAM("Run command: ps aux | grep adb");
    std::stringstream b;
    TinyProcessLib::Process process("ps aux | grep adb", "", [&](const char *bytes, size_t n) {
        ROS_INFO_STREAM(std::string(bytes, n));
        b << std::string(bytes, n);
    });

    int exit_status = process.get_exit_status();
    if (exit_status != 0) {
        ROS_INFO_STREAM("Exit status: " << exit_status);
        throw std::runtime_error("check_adb_server error");
    }

    return b.str().find("fork-server") != std::string::npos;
}

OpenstfInit::OpenstfInit(
        const std::string &adb_path,
        const std::string &minicap_libs_dir,
        const std::string &minicap_shared_libs_dir,
        const std::string &minitouch_libs_dir,
        const std::string &tmp_dir_name,
        const std::string &tmp_dir,
        const std::string &dev_dir) : m_adb_path(adb_path), m_dev_dir(dev_dir) {

    run_command({"/bin/rm", "-rf", tmp_dir});
    run_command({"/bin/mkdir", tmp_dir});

    bool adb_server_runnung = check_adb_server();
    ROS_INFO_STREAM("adb_server_runnung= " << adb_server_runnung);

    if (!adb_server_runnung) {
        run_command({adb_path, "start-server"});
    }

    run_command({adb_path, "wait-for-device"});

    auto abi = command_output_line(std::vector<std::string>{adb_path, "shell", "getprop", "ro.product.cpu.abi"});
    auto sdk = command_output_line(std::vector<std::string>{adb_path, "shell", "getprop", "ro.build.version.sdk"});
    auto release = command_output_line(
            std::vector<std::string>{adb_path, "shell", "getprop", "ro.build.version.release"});

    std::string pie;
    ROS_INFO_STREAM(sdk);
    if (std::stoi(sdk) < 16) {
        pie = "-nopie";
    }

    auto device_size = split(
            split(command_output_line(std::vector<std::string>{adb_path, "shell", "wm", "size"}), " ").back(), "x");
    m_width = stoi(device_size[0]);
    m_height = stoi(device_size[1]);

    run_command({"/bin/cp", minicap_libs_dir + "/" + abi + "/minicap" + pie, tmp_dir});
    run_command({"/bin/cp", minitouch_libs_dir + "/" + abi + "/minitouch" + pie, tmp_dir});

    auto minicap_so_path = minicap_shared_libs_dir + "/android-" + release + "/" + abi + "/minicap.so";
    if (!boost::filesystem::exists(minicap_so_path)) {
        minicap_so_path = minicap_shared_libs_dir + "/android-" + sdk + "/" + abi + "/minicap.so";
    }
    run_command({"/bin/cp", minicap_so_path, tmp_dir});

    run_command({adb_path, "push", tmp_dir, dev_dir});

    run_command({"/bin/rm", "-rf", tmp_dir});

    // TODO: add ports to config
    run_command({adb_path, "forward", "tcp:1313", "localabstract:minicap"});
    run_command({adb_path, "forward", "tcp:1111", "localabstract:minitouch"});
}

OpenstfInit::~OpenstfInit() {
    try {
        run_command({m_adb_path, "shell", "rm", "-rf", m_dev_dir});
    }
    catch (...) {
        //TODO: add some logging
    }
}

int OpenstfInit::height() {
    return m_height;
}

int OpenstfInit::width() {
    return m_width;
}
