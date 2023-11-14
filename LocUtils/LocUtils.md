#### 一、针对fmt报错的问题，fmt需要静态库链接到动态库，编译需要加-fPIC参数

1. 进入fmt库目录

2. ```powershell
   cd build
   ```

3. ```powershell
   sudo xargs rm <  install_manifest.txt     #卸载fmt库
   ```

4. ```powershell
   gedit CMakeLists.txt
   ```

5. //在该文件下添加

   ```cmake
   add_compile_options(-fPIC)
   ```

6. 运行如下指令

   ```powershell
   cd fmt
   rm-r build #删除原本的build目录
   mkdir build
   cd build
   cmake ..
   make
   sudo make install
   ```

#### 缺少cholmod.h: 没有那个文件或目录/usr/local/include/g2o/solvers/cholmod/linear_solver_cholmod.h:30:10: fatal error:

   ```cmake
   include_directories( "/usr/include/suitesparse")
   ```

#### 二、其他库链接到这个仓库需要链接 
```cmake
target_link_libraries(your_node glog gflags)
   
```



#### 三丶解决PLICP配准BUG
```c++

        Eigen::Quaterniond q(pcl_icp_ptr_->getFinalTransformation().cast<double>().block<3,3>(0,0));
        q.normalize();
        result_pose.setRotationMatrix(q.toRotationMatrix());    // 获取变换矩阵
        result_pose.translation()[0] = pcl_icp_ptr_->getFinalTransformation().cast<double>()(0,3);
        result_pose.translation()[1] = pcl_icp_ptr_->getFinalTransformation().cast<double>()(1,3);
        result_pose.translation()[2] = pcl_icp_ptr_->getFinalTransformation().cast<double>()(2,3);


```