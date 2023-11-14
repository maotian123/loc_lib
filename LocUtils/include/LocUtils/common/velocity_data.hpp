#pragma once

#include <deque>
#include <Eigen/Dense>

namespace LocUtils 
{
  class VelocityData 
  {
    public:
      struct LinearVelocity 
      {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
      };

      struct AngularVelocity 
      {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
      };

      double time = 0.0;
      LinearVelocity linear_velocity;
      AngularVelocity angular_velocity;
    
    public:
      static bool SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time);
      void TransformCoordinate(Eigen::Matrix4f transform_matrix);
      void NED2ENU(void);
  };
}
