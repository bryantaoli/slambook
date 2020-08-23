// Config类负责参数文件读取
#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h" 

namespace myslam 
{
class Config
{
private:
    static std::shared_ptr<Config> config_; 
    cv::FileStorage file_; // 读取yaml文件，且可以访问任意一个字段
    
    Config () {} // private constructor makes a singleton
public:
    ~Config();  // close the file when deconstructing 
    
    // set a new config file 
    static void setParameterFile( const std::string& filename ); 
    
    // access the parameter values
    template< typename T >
    static T get( const std::string& key ) // 模板类get函数获得任意类型参数值
    {
        return T( Config::config_->file_[key] );
    }
};
}

#endif