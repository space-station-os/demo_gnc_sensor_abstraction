#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tinyxml2.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <variant>
#include <thread>
#include <chrono>

using ParamType = std::variant<int, double, bool, std::string>;

class SensorNode : public rclcpp::Node
{
public:
    SensorNode()
    : Node("sensor_node")
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("demo_gnc_sensor_abstraction");

        // Load my_camera SDF file and set parameters
        my_camera = parseSDF(readFile(package_share_directory + "/sensors/mycamera.sdf"), "my_camera");
        // Load my_gps SDF file and set parameters
        my_gps = parseSDF(readFile(package_share_directory + "/sensors/mygps.sdf"), "gps_sensor");
        
        // Create publishers
        camera_publisher_ = this->create_publisher<std_msgs::msg::String>("camera_data", 10);
        gps_publisher_ = this->create_publisher<std_msgs::msg::String>("gps_data", 10);
        
        // Access update_rate for my_camera and set timer
        double camera_update_rate = getKey(my_camera, "update_rate");
        std::chrono::duration<double> interval_camera(1.0 / camera_update_rate);
        timer_my_camera = this->create_wall_timer(interval_camera, std::bind(&SensorNode::publish_camera_data, this));

        // Access update_rate for my_gps and set timer
        //double update_rate_gps = std::get<double>(my_gps.at("update_rate"));
        double gps_update_rate = getKey(my_gps, "update_rate");
        std::chrono::duration<double> interval_gps(1.0 / gps_update_rate);
        timer_my_gps = this->create_wall_timer(interval_gps, std::bind(&SensorNode::publish_gps_data, this));
        

        RCLCPP_INFO(this->get_logger(), "SensorNode initialized successfully.");
    }

private:

    double getKey(const std::map<std::string, ParamType>& params, const std::string& key)
    {
        auto it = params.find(key);
        if (it != params.end())
        {
            if (std::holds_alternative<double>(it->second))
            {
                return std::get<double>(it->second);
            }
            else if (std::holds_alternative<int>(it->second))
            {
                return static_cast<double>(std::get<int>(it->second));
            }
        }
        throw std::runtime_error("Update rate not found or wrong type");
    }
    // Publish my_camera data
    void publish_camera_data()
    {
	RCLCPP_INFO(this->get_logger(), "publish_camera_data()");
        for (const auto& [key, value] : my_camera) {
            std::visit([this, &key](auto&& arg) {
                std_msgs::msg::String msg;
                if constexpr (std::is_same_v<std::decay_t<decltype(arg)>, std::string>) {
                    msg.data = "Camera: " + key + ": " + arg;
                } else {
                    msg.data = "Camera: " + key + ": " + std::to_string(arg);
                }
                camera_publisher_->publish(msg);
            }, value);
        }
    }

    // Publish my_gps data
    void publish_gps_data()
    {
	RCLCPP_INFO(this->get_logger(), "publish_gps_data()");
        for (const auto& [key, value] : my_gps) {
            std::visit([this, &key](auto&& arg) {
                std_msgs::msg::String msg;
                if constexpr (std::is_same_v<std::decay_t<decltype(arg)>, std::string>) {
                    msg.data = "GPS: " + key + ": " + arg;
                } else {
                    msg.data = "GPS: " + key + ": " + std::to_string(arg);
                }
                gps_publisher_->publish(msg);
            }, value);
        }
    }
    
    std::string readFile(const std::string &filePath)
    {
        std::ifstream file(filePath);
        if (!file) {
            throw std::runtime_error("Failed to open file: " + filePath);
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }

    std::map<std::string, ParamType> parseSDF(const std::string &sdfContent, const std::string &sensorName)
    {
        tinyxml2::XMLDocument doc;
        doc.Parse(sdfContent.c_str());
        tinyxml2::XMLElement *root = doc.RootElement();
        if (!root) {
            throw std::runtime_error("Failed to parse SDF: No root element.");
        }

        std::map<std::string, ParamType> params;
        tinyxml2::XMLElement *model = root->FirstChildElement("model");
        if (model) {
            tinyxml2::XMLElement *link = model->FirstChildElement("link");
            if (link) {
                tinyxml2::XMLElement *sensor = link->FirstChildElement("sensor");
                while (sensor) {
                    const char *type = sensor->Attribute("type");
                    const char *name = sensor->Attribute("name");

                    if (type && name && sensorName == name) {
                        tinyxml2::XMLElement *child = sensor->FirstChildElement();
                        while (child) {
                            const char *key = child->Value();
                            const char *value = child->GetText();

                            if (value != nullptr) {
                                // Determine variable type and add to params
                                std::string value_str(value);
                                if (value_str == "true" || value_str == "false") {
                                    params[key] = (value_str == "true");
                                } else if (value_str.find('.') != std::string::npos) {
                                    params[key] = std::stod(value_str);
                                } else {
                                    try {
                                        params[key] = std::stoi(value_str);
                                    } catch (...) {
                                        params[key] = value_str;
                                    }
                                }
                            }
                            child = child->NextSiblingElement();
                        }
                        break;
                    }
                    sensor = sensor->NextSiblingElement("sensor");
                }
            }
        }
        return params;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gps_publisher_;
    rclcpp::TimerBase::SharedPtr timer_my_camera;
    rclcpp::TimerBase::SharedPtr timer_my_gps;

    std::map<std::string, ParamType> my_camera;
    std::map<std::string, ParamType> my_gps;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



