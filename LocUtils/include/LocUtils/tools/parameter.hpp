/*
 * @Author: wei-zhifei afeii2@126.com
 * @Date: 2023-10-25 21:10:12
 * @LastEditors: wei-zhifei afeii2@126.com
 * @LastEditTime: 2023-10-26 18:37:41
 * @FilePath: /LocUtils/include/LocUtils/tools/parameter.hpp
 * @Description: ---
 */
#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace LocUtils
{
    class Parameters
    {
    private:
        
        std::string path_;

    public:
        Parameters(std::string path):path_(path)
        {
        }
        ~Parameters(){}

        /**
         * @description: 获取参数值，通过"/"来区分嵌套关系，若读取失败则返回默认值。
         * @param {string} param_name 例如 "lio/method" "blog/name"
         * @param { T } default_value 该参数类型对应的默认值
         * @return { T } 返回读取到的参数值或默认值 //
         */
        template <typename T>
        T GetParamFromYaml(const std::string param_name, T default_value)
        {
            std::vector<std::string> names;
            std::string buf;
            std::istringstream in(param_name);
            T return_value;
            YAML::Node config = YAML::LoadFile(path_);

            while (getline(in, buf, '/'))
            {
                names.emplace_back(buf);
            }

            // std::cout << names[0] << " " << names[1] << " " << names[2] << std::endl;
            // std::cout << "-----------------" << std::endl;
            // std::cout << config[names[0]][names[1]][names[2]] << std::endl;
            // std::cout << "for:" << std::endl;
            for (auto& pa : names)
            {
                if (config[pa])
                {
                    if (&pa == &names.back())
                    {
                        return_value = config[pa].as<T>();
                        // std::cout << "return: " << return_value << std::endl;
                    }
                    else
                    {
                        config = config[pa];
                        // std::cout << "temp: " << temp_config << std::endl;
                    }
                }
                else
                {
                    LOG(ERROR) << "[" << pa << "] param is not found.";
                    return_value = default_value;
                    break;
                }
            }
            return return_value;
        }
    };
}