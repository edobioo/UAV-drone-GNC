#include "uav.hpp"

#include <cmath>
#include <iostream>
#include <string>

void UAV::init(void)
{
   /* ---------------------------------------------------------------------- */
   // Initialization sequence UAV parameter, SI units
   // Mass
   p_mass = 1.0;

   // Inertia matrix
   Eigen::Vector3d inertia_diag;
   inertia_diag << 0.00556, 0.00556, 0.01063;
   p_inertia = inertia_diag.asDiagonal();
   m_inertia_inv = p_inertia.inverse();

   // Motors
   // M1
   Motor& M1 = m_motors[0];
   M1.p_max_thrust = 5.0;
   M1.p_arm_to_COM = 0.19;
   M1.p_position_to_COM[0] = M1.p_arm_to_COM / std::sqrt(2);    
   M1.p_position_to_COM[1] = M1.p_arm_to_COM / std::sqrt(2);    
   M1.p_position_to_COM[2] = 0.0;   

   // M2
   Motor& M2 = m_motors[1];
   M2.p_max_thrust = 5.0;
   M2.p_arm_to_COM = 0.19;
   M2.p_position_to_COM[0] = - M2.p_arm_to_COM / std::sqrt(2);    
   M2.p_position_to_COM[1] = M2.p_arm_to_COM / std::sqrt(2);    
   M2.p_position_to_COM[2] = 0.0;  

   // M3
   Motor& M3 = m_motors[2];
   M3.p_max_thrust = 5.0;
   M3.p_arm_to_COM = 0.19;
   M3.p_position_to_COM[0] = - M3.p_arm_to_COM / std::sqrt(2);    
   M3.p_position_to_COM[1] = - M3.p_arm_to_COM / std::sqrt(2);    
   M3.p_position_to_COM[2] = 0.0;  

   // M4
   Motor& M4 = m_motors[3];
   M4.p_max_thrust = 5.0;
   M4.p_arm_to_COM = 0.19;
   M4.p_position_to_COM[0] = M4.p_arm_to_COM / std::sqrt(2);    
   M4.p_position_to_COM[1] = - M4.p_arm_to_COM / std::sqrt(2);    
   M4.p_position_to_COM[2] = 0.0;   

   // Thrust to body force matrix
   p_TB_F << 0.0, 0.0, 0.0, 0.0, 
             0.0, 0.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 1.0;

   // Thrust to body torque matrix
   p_TB_T(0, 0) = M1.p_position_to_COM[1];
   p_TB_T(0, 1) = M2.p_position_to_COM[1];
   p_TB_T(0, 2) = M3.p_position_to_COM[1];
   p_TB_T(0, 3) = M4.p_position_to_COM[1];
   p_TB_T(1, 0) = -M1.p_position_to_COM[0];
   p_TB_T(1, 1) = -M2.p_position_to_COM[0];
   p_TB_T(1, 2) = -M3.p_position_to_COM[0];
   p_TB_T(1, 3) = -M4.p_position_to_COM[0];
   p_TB_T(2, 0) = 0.0; // TBC
   p_TB_T(2, 1) = 0.0; // TBC
   p_TB_T(2, 2) = 0.0; // TBC
   p_TB_T(2, 3) = 0.0; // TBC

   /* ---------------------------------------------------------------------- */
   // Initialization UAV state, SI units
   m_position_ENU << 0.0, 0.0, 0.0;
   m_velocity_ENU << 0.0, 0.0, 0.0;
   m_quat_ENU_body.w() = 1.0;
   m_quat_ENU_body.x() = 0.0;
   m_quat_ENU_body.y() = 0.0;
   m_quat_ENU_body.z() = 0.0;
   m_omega_body << 0.0, 0.0, 0.0;

   // Set constant input (TBC)
   M1.m_thrust = 2.499998;
   M2.m_thrust = 2.499998;
   M3.m_thrust = 2.5;
   M4.m_thrust = 2.5;
}

UAV::State UAV::derivative(const State& state,
                           const Input& input)
{
   // Extract single states from state vector
   const Eigen::Vector3d velocity_ENU = state.segment<3>(3);
   const Eigen::Quaterniond quat_ENU_body(state(6), state(7), 
                                          state(8), state(9)); // scalar first
   const Eigen::Vector3d omega_body = state.segment<3>(10);

   // Rotation matrix from quaternion
   const Eigen::Matrix3d R_ENU_body = 
                         quat_ENU_body.normalized().toRotationMatrix();

   // Pure quaternion from omega
   const Eigen::Quaterniond quat_omega_body(0.0,            // w
                                            omega_body(0),  // x 
                                            omega_body(1),  // y
                                            omega_body(2)); // z

   // Compute state dot
   State state_dot;

   // Velocity
   state_dot.segment<3>(0) = velocity_ENU; 
   // Acceleration
   state_dot.segment<3>(3) = (R_ENU_body * p_TB_F * input) / p_mass + 
                             c_gravity_ENU;
   // Quaternion dot
   Eigen::Quaterniond quat_ENU_body_dot = quat_ENU_body * quat_omega_body;
   quat_ENU_body_dot.coeffs() *= 0.5; // multiply by scalar
   state_dot.segment<4>(6) << quat_ENU_body_dot.w(), 
                              quat_ENU_body_dot.x(),
                              quat_ENU_body_dot.y(),
                              quat_ENU_body_dot.z();
   // Omega dot
   state_dot.segment<3>(10) = m_inertia_inv * 
                              (p_TB_T * input - 
                               omega_body.cross(p_inertia * omega_body));

   return state_dot;
}

void UAV::integrate(const TimingType& timing)
{
   // Build state vector
   State state;
   state.segment<3>(0) = m_position_ENU;
   state.segment<3>(3) = m_velocity_ENU;
   state.segment<4>(6) << m_quat_ENU_body.w(),
                          m_quat_ENU_body.x(),
                          m_quat_ENU_body.y(),
                          m_quat_ENU_body.z();
   state.segment<3>(10) = m_omega_body;

   // Build input vector
   Input input;
   input(0) = m_motors[0].m_thrust;
   input(1) = m_motors[1].m_thrust;
   input(2) = m_motors[2].m_thrust;
   input(3) = m_motors[3].m_thrust;
   
   // Define lambda for derivative
   const auto lambda_derivative = 
      [this](const State& state, const Input& input) -> State {
         return derivative(state, input);
      };

   State state_new = m_rk4.step(timing.sim_step,
                                state, input, 
                                lambda_derivative); 

   // Update new state members
   m_position_ENU = state_new.segment<3>(0);
   m_velocity_ENU = state_new.segment<3>(3);
   m_quat_ENU_body.w() = state_new(6);
   m_quat_ENU_body.x() = state_new(7);
   m_quat_ENU_body.y() = state_new(8);
   m_quat_ENU_body.z() = state_new(9);
   m_quat_ENU_body.normalize(); // normalize quaternion
   m_omega_body = state_new.segment<3>(10);
}

void UAV::print(const TimingType& timing)
{
   // Open file and write header at first call
   if (m_flag_to_init_print)
   {
      m_flag_to_init_print = false;

      const std::string filename = "./outputs/uav" +
                                    std::to_string(m_ID) + 
                                    "_log.csv";

      m_outfile.open(filename);

      if (!m_outfile)
         throw std::runtime_error("Error in opening output file: " + filename);

      // Write header
      m_outfile << "Time" << ","
                << "Position_ENU[0]" << ","
                << "Position_ENU[1]" << ","
                << "Position_ENU[2]" << ","
                << "Velocity_ENU[0]" << ","
                << "Velocity_ENU[1]" << ","
                << "Velocity_ENU[2]" << ","
                << "Quaternion_ENU_body[0]" << ","
                << "Quaternion_ENU_body[1]" << ","
                << "Quaternion_ENU_body[2]" << ","
                << "Quaternion_ENU_body[3]" << ","
                << "Omega_body[0]" << ","
                << "Omega_body[1]" << ","
                << "Omega_body[2]" << "\n";
   }

   // Write log 
   m_outfile << timing.time << ","
             << m_position_ENU(0) << ","
             << m_position_ENU(1) << ","
             << m_position_ENU(2) << ","
             << m_velocity_ENU(0) << ","
             << m_velocity_ENU(1) << ","
             << m_velocity_ENU(2) << ","
             << m_quat_ENU_body.w() << ","
             << m_quat_ENU_body.x() << ","
             << m_quat_ENU_body.y() << ","
             << m_quat_ENU_body.z() << ","
             << m_omega_body(0) << ","
             << m_omega_body(1) << ","
             << m_omega_body(2) << "\n";
}

