#include <string>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include "print.h"

class YamlParse {

    public:
        explicit YamlParse(const std::string &config_path, bool fail_if_not_found = true) : config_path_(config_path) {
            if (!fail_if_not_found && !boost::filesystem::exists(config_path)) {
                config = nullptr;
                return;
            }
            if (!boost::filesystem::exists(config_path)) {
                PRINT_ERROR(RED "unable to open the configuration file!\n%s\n" RESET, config_path.c_str());
                std::exit(EXIT_FAILURE);
            }

            config = std::make_shared<cv::FileStorage>(config_path, cv::FileStorage::READ);
            if (!fail_if_not_found && !config->isOpened()) {
                config = nullptr;
                return;
            }
            if (!config->isOpened()) {
                PRINT_ERROR(RED "unable to open the configuration file!\n%s\n" RESET, config_path.c_str());
                std::exit(EXIT_FAILURE);
            }
        }

        void set_node_handler(std::shared_ptr<ros::NodeHandle> nh_) { this->nh = nh_; }

        std::string get_config_folder() { return config_path_.substr(0, config_path_.find_last_of('/')) + "/"; }

        bool successful() const { return all_params_found_successfully; }

    private:
        std::string config_path_;

        std::shared_ptr<cv::FileStorage> config;

        bool all_params_found_successfully = true;

        std::shared_ptr<ros::NodeHandle> nh;



};