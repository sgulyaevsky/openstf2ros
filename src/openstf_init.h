#pragma once

#include <string>

class OpenstfInit {
public:
    const int minicap_port = 1313;
    const int minitouch_port = 1111;

    OpenstfInit(OpenstfInit const &) = delete;

    OpenstfInit &operator=(OpenstfInit const &) = delete;

    OpenstfInit(
            const std::string &adb_path,
            const std::string &minicap_libs_dir,
            const std::string &minicap_shared_libs_dir,
            const std::string &minitouch_libs_dir,
            const std::string &tmp_dir_name,
            const std::string &tmp_dir,
            const std::string &dev_dir
    );

    ~OpenstfInit();

    int height();

    int width();

private:
    const std::string m_adb_path;
    const std::string m_dev_dir;

    int m_width;
    int m_height;
};

