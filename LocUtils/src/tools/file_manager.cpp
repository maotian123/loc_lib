//
// Created by Tian on 2023/10/01.
//

#include "LocUtils/tools/file_manager.hpp"

#include <boost/filesystem.hpp>
#include <glog/logging.h>

namespace LocUtils
{
    bool FileManager::IsFileExist(std::string file_path)
    {
        if (boost::filesystem::exists(file_path) == 0)
        {
            LOG(INFO) << file_path << " dose not exists." << std::endl;
            return false;
        }
        
        LOG(INFO) << file_path << " exists." << std::endl;
        return true;
    }

    bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path) 
    {
        ofs.close();
        boost::filesystem::remove(file_path.c_str());

        ofs.open(file_path.c_str(), std::ios::out);
        if (!ofs) {
            LOG(INFO) << "无法生成文件: " << std::endl << file_path << std::endl << std::endl;
            return false;
        }

        return true;
    }

    bool FileManager::InitDirectory(std::string directory_path, std::string use_for) 
    {
        if (boost::filesystem::is_directory(directory_path)) {
            boost::filesystem::remove_all(directory_path);
        }

        return CreateDirectory(directory_path, use_for);
    }

    bool FileManager::CreateDirectory(std::string directory_path, std::string use_for) 
    {
        if (!boost::filesystem::is_directory(directory_path)) {
            boost::filesystem::create_directory(directory_path);
        }

        if (!boost::filesystem::is_directory(directory_path)) {
            LOG(INFO) << "CANNOT create directory " << std::endl << directory_path << std::endl << std::endl;
            return false;
        }

        std::cout << use_for << " output path:" << std::endl << directory_path << std::endl << std::endl;
        return true;
    }

    bool FileManager::CreateDirectory(std::string directory_path) 
    {
        if (!boost::filesystem::is_directory(directory_path)) {
            boost::filesystem::create_directory(directory_path);
        }

        if (!boost::filesystem::is_directory(directory_path)) {
            LOG(INFO) << "CANNOT create directory " << std::endl << directory_path << std::endl << std::endl;
            return false;
        }

        return true;
    }

    //遍历获取指定路径下文件
    bool FileManager::GetFilesNameFromDirectory(std::vector<std::string>& vecFiles, const std::string& sPath)
    {
        try
        {
            vecFiles.clear();
            boost::filesystem::path path(sPath);
            for (const auto& iter : boost::filesystem::directory_iterator(path))
            {
                if (boost::filesystem::is_directory(iter.path()))//过滤子文件夹
                    continue;

                std::string sFile = iter.path().filename().string().c_str();
                vecFiles.push_back(sFile);
            }
            // if (SortFileList(vecFiles, "pcd") == false)
            //     return false;
            return true;
        }
        catch (const std::exception& error)
        {
            std::string sError = error.what();
        }
        return false;
    }
    //遍历获取指定路径下文件(按数字大小顺序存储在vector中)
    bool FileManager::GetFilesNameFromDirectory(std::vector<std::string>& vecFiles, const std::string& sPath, bool order)
    {
        try
        {
            vecFiles.clear();
            boost::filesystem::path path(sPath);
            for (const auto& iter : boost::filesystem::directory_iterator(path))
            {
                if (boost::filesystem::is_directory(iter.path()))//过滤子文件夹
                    continue;

                std::string sFile = iter.path().filename().string().c_str();
                vecFiles.push_back(sFile);
            }
            if (false == SortFileList(vecFiles, "pcd"))
                return false;
            return true;
        }
        catch (const std::exception& error)
        {
            std::string sError = error.what();
        }
        return false;
    }
} // namespace name
