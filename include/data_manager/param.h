#ifndef DATA_MANAGER_PARAM_H_
#define DATA_MANAGER_PARAM_H_

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <fstream>
#include "json.hpp"

class Param {
public:
    Param(const std::string& json_path) { load(json_path); }
    static std::shared_ptr<Param> get_instance() {
        static std::shared_ptr<Param> instance(new Param());
        return instance;
    }
    
    bool load(const std::string&);
    void dump(const std::string&);

    nlohmann::json& operator[](const std::string&);

    static void from_json(const nlohmann::json& j, cv::Mat& p);
    static void to_json(nlohmann::json& j, const cv::Mat& p);

private:
    Param() { load(default_path_); }
    Param(const Param&) = delete;
    Param& operator=(const Param&) = delete;

private:
    nlohmann::json params_;
    std::string default_path_ = "/etc/openrm_engineering/param.json";


};

// namespace JsonMat {

// // 实现Mat与Json对象的转化，将Json对象转化为Mat对象
// void from_json(const nlohmann::json& j, cv::Mat& p);

// // 实现Mat与Json对象的转化，将Mat对象转化为Json对象
// void to_json(nlohmann::json& j, const cv::Mat& p);

// }


#endif