# Compute attitude from static accelerometer sample using alignment quaternion
# q_out -> output quaternion, q_body_ENU passive
# f_meas -> specific force reading from accelerometer
# accel -> struct with accelerometer parameters
function [q_out] = accel_to_attitude(f_meas, accel)

   # Normalize accelerometer sample
   if (norm(f_meas) != 0)
      f_meas_norm = f_meas / norm(f_meas);
   else
      f_meas_norm = zeros(3, 1);
   endif

   # The equivalent reading in ENU frame is [0; 0; 1]
   f_ENU = [0; 0; 1];

   # Extract components
   fx = f_meas_norm(1);
   fy = f_meas_norm(2);
   fz = f_meas_norm(3);

   # Compute quaternion
   q_out = zeros(4, 1);
   if (fz >= 0)
      q_out(1) = sqrt((fz + 1) / 2);
      q_out(2) = fy / sqrt(2 * (fz + 1));
      q_out(3) = - fx / sqrt(2 * (fz + 1));
      q_out(4) = 0;
   else
      q_out(1) = fy / (sqrt(2 * (1 - fz)));
      q_out(2) = sqrt((1 - fz) / 2);
      q_out(3) = 0;
      q_out(4) = fx / sqrt(2 * (1 - fz));
   endif

   # Normalize
   q_out = q_out / norm(q_out);

end
