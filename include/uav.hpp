#pragma once
#include "typedefs.hpp"
#include "rk4.hpp"

#include <cstddef>
#include <string>
#include <array>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

class UAV
{
   private:
      // Body reference frame === COM
      /* Body reference frame convention
         z UP, motor enum counterclockwise
         M1 -> +x, +y
         M2 -> -x, +y
         M3 -> -x, -y
         M4 -> +x, -y
      */
      // Objects
      RungeKutta4<13, 4> m_rk4; // integrator

      // Structs
      struct Motor
      {
         double p_arm_to_COM;
         Eigen::Vector3d p_position_to_COM;
         int p_yaw_sign;
         double p_max_thrust;
         double m_thrust;
      };

      // Alias
      using State = Eigen::Vector<double, 13>;
      using Input = Eigen::Vector<double, 4>;

      // Static obj ID counter
      static inline std::size_t s_current_ID = 0;

      // Constants
      const Eigen::Vector3d c_gravity_ENU{0.0, 0.0, -9.80665};

      // Parameters
      double p_mass; 
      Eigen::Matrix3d p_inertia; // body frame
      Eigen::Matrix<double, 3, 4> p_TB_F; // thrust to body force
      Eigen::Matrix<double, 3, 4> p_TB_T; // thrust to body torque

      // Variables
      std::size_t m_ID;

      std::array<Motor, 4> m_motors;

      Eigen::Matrix3d m_inertia_inv;

      Eigen::Vector3d m_position_ENU;
      Eigen::Vector3d m_velocity_ENU;
      Eigen::Quaterniond m_quat_ENU_body; // passive, maps body to ENU
      Eigen::Vector3d m_omega_body;

      bool m_flag_to_init_print = true;
      std::ofstream m_outfile;

      // Private methods
      State derivative(const State& state,
                       const Input& input);
      
   public:
      // Constructor
      UAV() : m_ID(++s_current_ID) {};

      void init(void);
      void integrate(const TimingType& timing);
      void print(const TimingType& timing);
};
