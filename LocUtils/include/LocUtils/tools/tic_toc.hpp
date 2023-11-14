//
// Created by Tian on 2023/10/01.
//

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

namespace LocUtils
{
    class TicToc 
    {
        public:
            TicToc() {
                tic();
            }

            void tic() {
                start = std::chrono::system_clock::now();
            }

            double toc() {
                end = std::chrono::system_clock::now();
                std::chrono::duration<double> elapsed_seconds = end - start;
                start = std::chrono::system_clock::now();
                return elapsed_seconds.count();
            }

        private:
            std::chrono::time_point<std::chrono::system_clock> start, end;
    };
} 
