#include <string>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include "print.h"
#include "colors.h"
#include "quat_ops.h"

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



        
        template <class T> void parse_config(const std::string &node_name, T &node_result, bool required = true) {
            if (nh != nullptr && nh->getParam(node_name, node_result)) {
            PRINT_INFO(GREEN "overriding node " BOLDGREEN "%s" RESET GREEN " with value from ROS!\n" RESET, node_name.c_str());
            nh->param<T>(node_name, node_result);
            return;
            }
            // Else we just parse from the YAML file!
            parse_config_yaml(node_name, node_result, required);
        }

        template <class T> void parse_config_yaml(const std::string &node_name, T &node_result, bool required = true) {

            // Directly return if the config hasn't been opened
            if (config == nullptr)
            return;

            // Else lets get the one from the config
            try {
            parse(config->root(), node_name, node_result, required);
            } catch (...) {
            PRINT_WARNING(YELLOW "unable to parse %s node of type [%s] in the config file!\n" RESET, node_name.c_str(),
                            typeid(node_result).name());
            all_params_found_successfully = false;
            }
        }





    private:


          template <class T> void parse(const cv::FileNode &file_node, const std::string &node_name, T &node_result, bool required = true) {

    // Check that we have the requested node
    if (!node_found(file_node, node_name)) {
      if (required) {
        PRINT_WARNING(YELLOW "the node %s of type [%s] was not found...\n" RESET, node_name.c_str(), typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("the node %s of type [%s] was not found (not required)...\n", node_name.c_str(), typeid(node_result).name());
      }
      return;
    }

    // Now try to get it from the config
    try {
      file_node[node_name] >> node_result;
    } catch (...) {
      if (required) {
        PRINT_WARNING(YELLOW "unable to parse %s node of type [%s] in the config file!\n" RESET, node_name.c_str(),
                      typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("unable to parse %s node of type [%s] in the config file (not required)\n", node_name.c_str(),
                    typeid(node_result).name());
      }
    }
  }

    static bool node_found(const cv::FileNode &file_node, const std::string &node_name) {
            bool found_node = false;
            for (const auto &item : file_node) {
            if (item.name() == node_name) {
                found_node = true;
            }
            }
            return found_node;
        }

  /**
   * @brief Custom parser for booleans (0,false,False,FALSE=>false and 1,true,True,TRUE=>false)
   * @param file_node OpenCV file node we will get the data from
   * @param node_name Name of the node
   * @param node_result Resulting value (should already have default value in it)
   * @param required If this parameter is required by the user to set
   */
  void parse(const cv::FileNode &file_node, const std::string &node_name, bool &node_result, bool required = true) {

    // Check that we have the requested node
    if (!node_found(file_node, node_name)) {
      if (required) {
        PRINT_WARNING(YELLOW "the node %s of type [%s] was not found...\n" RESET, node_name.c_str(), typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("the node %s of type [%s] was not found (not required)...\n", node_name.c_str(), typeid(node_result).name());
      }
      return;
    }

    // Now try to get it from the config
    try {
      if (file_node[node_name].isInt() && (int)file_node[node_name] == 1) {
        node_result = true;
        return;
      }
      if (file_node[node_name].isInt() && (int)file_node[node_name] == 0) {
        node_result = false;
        return;
      }
      // NOTE: we select the first bit of text as there can be a comment afterwards
      // NOTE: for example we could have "key: true #comment here..." where we only want "true"
      std::string value;
      file_node[node_name] >> value;
      value = value.substr(0, value.find_first_of('#'));
      value = value.substr(0, value.find_first_of(' '));
      if (value == "1" || value == "true" || value == "True" || value == "TRUE") {
        node_result = true;
      } else if (value == "0" || value == "false" || value == "False" || value == "FALSE") {
        node_result = false;
      } else {
        PRINT_WARNING(YELLOW "the node %s has an invalid boolean type of [%s]\n" RESET, node_name.c_str(), value.c_str());
        all_params_found_successfully = false;
      }
    } catch (...) {
      if (required) {
        PRINT_WARNING(YELLOW "unable to parse %s node of type [%s] in the config file!\n" RESET, node_name.c_str(),
                      typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("unable to parse %s node of type [%s] in the config file (not required)\n", node_name.c_str(),
                    typeid(node_result).name());
      }
    }
  }

  /**
   * @brief Custom parser for camera extrinsic 3x3 transformations
   * @param file_node OpenCV file node we will get the data from
   * @param node_name Name of the node
   * @param node_result Resulting value (should already have default value in it)
   * @param required If this parameter is required by the user to set
   */
  void parse(const cv::FileNode &file_node, const std::string &node_name, Eigen::Matrix3d &node_result, bool required = true) {

    // Check that we have the requested node
    if (!node_found(file_node, node_name)) {
      if (required) {
        PRINT_WARNING(YELLOW "the node %s of type [%s] was not found...\n" RESET, node_name.c_str(), typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("the node %s of type [%s] was not found (not required)...\n", node_name.c_str(), typeid(node_result).name());
      }
      return;
    }

    // Now try to get it from the config
    node_result = Eigen::Matrix3d::Identity();
    try {
      for (int r = 0; r < (int)file_node[node_name].size() && r < 3; r++) {
        for (int c = 0; c < (int)file_node[node_name][r].size() && c < 3; c++) {
          node_result(r, c) = (double)file_node[node_name][r][c];
        }
      }
    } catch (...) {
      if (required) {
        PRINT_WARNING(YELLOW "unable to parse %s node of type [%s] in the config file!\n" RESET, node_name.c_str(),
                      typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("unable to parse %s node of type [%s] in the config file (not required)\n", node_name.c_str(),
                    typeid(node_result).name());
      }
    }
  }

  /**
   * @brief Custom parser for camera extrinsic 4x4 transformations
   * @param file_node OpenCV file node we will get the data from
   * @param node_name Name of the node
   * @param node_result Resulting value (should already have default value in it)
   * @param required If this parameter is required by the user to set
   */
  void parse(const cv::FileNode &file_node, const std::string &node_name, Eigen::Matrix4d &node_result, bool required = true) {

    // See if we need to flip the node name
    std::string node_name_local = node_name;
    if (node_name == "T_cam_imu" && !node_found(file_node, node_name)) {
      PRINT_INFO("parameter T_cam_imu not found, trying T_imu_cam instead (will return T_cam_imu still)!\n");
      node_name_local = "T_imu_cam";
    } else if (node_name == "T_imu_cam" && !node_found(file_node, node_name)) {
      PRINT_INFO("parameter T_imu_cam not found, trying T_cam_imu instead (will return T_imu_cam still)!\n");
      node_name_local = "T_cam_imu";
    }

    // Check that we have the requested node
    if (!node_found(file_node, node_name_local)) {
      if (required) {
        PRINT_WARNING(YELLOW "the node %s of type [%s] was not found...\n" RESET, node_name_local.c_str(), typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("the node %s of type [%s] was not found (not required)...\n", node_name_local.c_str(), typeid(node_result).name());
      }
      return;
    }

    // Now try to get it from the config
    node_result = Eigen::Matrix4d::Identity();
    try {
      for (int r = 0; r < (int)file_node[node_name_local].size() && r < 4; r++) {
        for (int c = 0; c < (int)file_node[node_name_local][r].size() && c < 4; c++) {
          node_result(r, c) = (double)file_node[node_name_local][r][c];
        }
      }
    } catch (...) {
      if (required) {
        PRINT_WARNING(YELLOW "unable to parse %s node of type [%s] in the config file!\n" RESET, node_name.c_str(),
                      typeid(node_result).name());
        all_params_found_successfully = false;
      } else {
        PRINT_DEBUG("unable to parse %s node of type [%s] in the config file (not required)\n", node_name.c_str(),
                    typeid(node_result).name());
      }
    }

    // Finally if we flipped the transform, get the correct value
    if (node_name_local != node_name) {
      Eigen::Matrix4d tmp(node_result);
      node_result = Inv_se3(tmp);
    }
  }




        std::string config_path_;

        std::shared_ptr<cv::FileStorage> config;

        bool all_params_found_successfully = true;

        std::shared_ptr<ros::NodeHandle> nh;



};