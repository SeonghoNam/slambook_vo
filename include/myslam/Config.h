#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h"

namespace myslam
{
    class Config
    {
        private:
        static std::shared_ptr<Config> _config;
        cv::FileStorage _file;
        Config() {}

        public:
        ~Config();

        static void setParameterFile(const std::string& filename);

        template<typename T>
        static T get(const std::string& key)
        {
            return T(Config::_config->_file[key]);
        }
    };
}

#endif