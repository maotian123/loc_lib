#include "LocUtils/model/reflactor/reflactor_proc.hpp"

#include <glog/logging.h>
namespace LocUtils
{
    ReflactorProcess::ReflactorProcess()
    {
        LOG(INFO) << "refactor prcess build ";
    }

    ReflactorProcess::ReflactorProcess(ReflactorOption option) :
        option_(option_)
    {
        LOG(INFO) << "refactor prcess build ";
    }


    void ReflactorProcess::AddScan(const Scan2d &scan)
    {
        
    }
}