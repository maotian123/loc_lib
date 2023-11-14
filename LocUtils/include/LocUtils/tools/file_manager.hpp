//
// Created by Tian on 2023/10/01.
//

#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

namespace LocUtils
{
    class FileManager
    {
        public:
            static bool IsFileExist(std::string file_path);
            static bool CreateFile(std::ofstream &ofs, std::string file_path);
            static bool InitDirectory(std::string directory_path, std::string use_for);
            static bool CreateDirectory(std::string directory_path, std::string use_for);
            static bool CreateDirectory(std::string directory_path);
            //排序方法（按扩展名前的int大小排序）
            static bool ComputePairNum(std::string pair1, std::string pair2)
            {
                pair1.erase(pair1.end() - 4, pair1.end());
                pair2.erase(pair2.end() - 4, pair2.end());
                int a1 = atoi(pair1.c_str());
                int a2 = atoi(pair2.c_str());

                return a1 < a2;
            }
            //排序
			static bool SortFileList(std::vector<std::string> files_lists, std::string file_type)
            {
                if (files_lists.empty())
                    return false;
                std::sort(files_lists.begin(), files_lists.end(), ComputePairNum);
                return true; 
            }
            //函数重载，可选文件名排序和不排序
			static bool GetFilesNameFromDirectory(std::vector<std::string>& vecFiles, const std::string& sPath);
            static bool GetFilesNameFromDirectory(std::vector<std::string>& vecFiles, const std::string& sPath, bool order);
    };
} // namespace name
