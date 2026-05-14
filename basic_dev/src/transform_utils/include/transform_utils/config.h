#pragma once
#include <memory>
#include <opencv2/core.hpp>
#include <iostream>
#include <cstdlib>

class Config
{
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {}

public:
    ~Config();

    static void setParameterFile(const std::string& filename);

    static cv::FileStorage& file()
    {
        if (config_ == nullptr)
        {
            std::cerr << "[Config] config is nullptr." << std::endl;
            std::exit(-1);
        }
        if (!config_->file_.isOpened())
        {
            std::cerr << "[Config] file is not opened." << std::endl;
            std::exit(-1);
        }
        return config_->file_;
    }
};