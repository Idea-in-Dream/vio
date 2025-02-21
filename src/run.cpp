#include <string>
#include <ros/ros.h>

#include "utils/yaml_parse.h"
#include "core/VioManagerOptions.h"
int main(int argc, char **argv) {
    std::string config_path = "config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    // 初始化ROS节点
    ros::init(argc, argv, "run_vio");
    auto nh = std::make_shared<ros::NodeHandle>("~");
    nh->param<std::string>("config_path", config_path, config_path);

    // 加载配置文件
    auto parser = std::make_shared<YamlParse>(config_path);
    parser->set_node_handler(nh);

    std::string verbosity = "DEBUG";
    parser->parse_config("verbosity", verbosity);
    Printer::setPrintLevel(verbosity);

    VioManagerOptions params;

    printf("Hello, World!\n");


}