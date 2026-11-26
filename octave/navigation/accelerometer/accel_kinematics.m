# Function to integrate accelerometer sample to obtain measured velocity and
# position (in ENU frame)
# p_meas -> output position integrated
# v_meas -> output velocity integrated
# f_meas -> input measured specific force
# q_body_ENU -> input quaternion mapping body to ENU
# p0 -> initial position in ENU frame
# v0 -> initial velocity in ENU frame
# accel -> struct with accelerometer parameters
function [p_meas, v_meas] = accel_kinematics(f_meas, q_body_ENU, p0, v0,  accel)

   # Compute R_body_ENU (DCM)
   R_body_ENU = quat_to_dcm(q_body_ENU);

   # Acceleration in ENU frame
   a_ENU = R_body_ENU * f_meas + accel.g_ENU;

   # Integrate acceleration
   v_meas = v0 + a_ENU * accel.sample_time;

   # Integrate velocity
   p_meas = p0 + v0 * accel.sample_time + 0.5 * a_ENU * accel.sample_time^2;

end
