#include "transform_utils/config.h"

std::shared_ptr<Config> Config::config_ = nullptr;

void Config::setParameterFile(const std::string& filename)
{
    if (config_ == nullptr)
        config_ = std::shared_ptr<Config>(new Config);
    try
    {config_->file_.open(filename, cv::FileStorage::READ);}
    
    catch (const cv::Exception& e)
    {
        std::cerr << "[Config] OpenCV exception while opening file: " << filename << std::endl;
        std::cerr << e.what() << std::endl;
        std::exit(-1);
    }

    if (!config_->file_.isOpened())
    {
        std::cerr << "[Config] Cannot open config file: " << filename << std::endl;
        std::exit(-1);
    }

    /*std::cout << "[Config] Opened config file: " << filename << std::endl;

    cv::FileNode root = config_->file_.root();
    std::cout << "[Config] root empty? " << root.empty() << std::endl;

    cv::FileNode first = config_->file_.getFirstTopLevelNode();
    std::cout << "[Config] first top-level node empty? " << first.empty() << std::endl;*/
}

Config::~Config()
{
    if (file_.isOpened())
        file_.release();
}