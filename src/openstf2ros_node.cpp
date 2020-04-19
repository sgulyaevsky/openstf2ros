#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include "openstf_init.h"
#include "minicap.h"
#include "minitouch.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "openstf2ros_node");

    ros::NodeHandle node_handle;

    // read parameters
    ros::NodeHandle private_node_handle("~");
    std::string adb_path;
    private_node_handle.getParam("adb_path", adb_path);

    bool run_minicap;
    private_node_handle.getParam("run_minicap", run_minicap);
    std::string minicap_libs_dir;
    private_node_handle.getParam("minicap_libs_dir", minicap_libs_dir);
    std::string minicap_shared_libs_dir;
    private_node_handle.getParam("minicap_shared_libs_dir", minicap_shared_libs_dir);
    int minicap_downscale_factor;
    private_node_handle.getParam("minicap_downscale_factor", minicap_downscale_factor);

    bool run_minitouch;
    private_node_handle.getParam("run_minitouch", run_minitouch);
    std::string minitouch_libs_dir;
    private_node_handle.getParam("minitouch_libs_dir", minitouch_libs_dir);

    std::string tmp_dir_name;
    private_node_handle.getParam("tmp_dir_name", tmp_dir_name);

    std::string tmp_dir = "/tmp/" + tmp_dir_name;
    std::string dev_dir = "/data/local/tmp/" + tmp_dir_name;

    try {
        // init
        OpenstfInit openstf_init(
                adb_path,
                minicap_libs_dir,
                minicap_shared_libs_dir,
                minitouch_libs_dir,
                tmp_dir_name,
                tmp_dir,
                dev_dir
        );

        std::unique_ptr<Minicap> minicap_ptr;
        if (run_minicap) {
            minicap_ptr = std::unique_ptr<Minicap>(new Minicap(
                    node_handle, 
                    openstf_init.width(), openstf_init.height(), 
                    2, 
                    adb_path, dev_dir,
                    openstf_init.minicap_port));
        }

        std::unique_ptr<Minitouch> minitouch_ptr;
        if (run_minitouch) {
            minitouch_ptr = std::unique_ptr<Minitouch>(new Minitouch(
                    node_handle, 
                    adb_path, dev_dir, 
                    openstf_init.minitouch_port));
        }

        ros::Rate loop_rate(1000);
        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    catch (std::exception &e) {
        ROS_ERROR_STREAM(e.what());
    }

    return 0;
}