# Quaternion kinematics from gyro -> the function integrates angular velocity
# to obtain quaternion using exponential map in SO(3)
# q_out -> output quaternion
# q_in -> unit q_body_ENU (Hamilton, scalar first), mapping body to ENU
# omega_in -> input angular velocity
# dt -> integration step
function [q_out] = gyro_kinematics(q_in, omega_in, dt)

   # Extract vectorial part
   q_vec = (omega_in * dt) / 2;
   q_vec_norm = sqrt(q_vec(1)^2 + q_vec(2)^2 + q_vec(3)^2);

   # Compute increment quaternion (pure quaternion formula)
   q_delta = zeros(4, 1);
   q_delta(1) = cos(q_vec_norm);

   if (q_vec_norm > 1e-6)
      # sin_gain = sin(norm(v)) / norm(v)
      sin_gain = (sin(q_vec_norm)) / q_vec_norm;
   else
      # Taylor expansion sin(norm(v)) / norm(v)
      sin_gain = 1 - q_vec_norm^2 / 6 + q_vec_norm^4 / 120;
   endif

   q_delta(2 : 4) = q_vec * sin_gain;

   # Compute quaternion product and normalize
   q_out = quat_prod(q_in, q_delta);
   q_out = q_out / sqrt(q_out(1)^2 + q_out(2)^2 + q_out(3)^2 + q_out(4)^2);

end
