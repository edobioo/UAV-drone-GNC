# Compute DCM from quaternion
# R_out -> R_source_destination; v_destination = R_out * v_source
# q_in -> unit q_source_destination, Hamilton scalar first
function [R_out] = quat_to_dcm(q_in)

   # Extract components
   q0 = q_in(1);
   q1 = q_in(2);
   q2 = q_in(3);
   q3 = q_in(4);

   R_out = zeros(3, 3);
   R_out(1, 1) = q0^2 + q1^2 - q2^2 - q3^2;
   R_out(1, 2) = 2 * (q1 * q2 - q0 * q3);
   R_out(1, 3) = 2 * (q1 * q3 + q0 * q2);
   R_out(2, 1) = 2 * (q1 * q2 + q0 * q3);
   R_out(2, 2) = q0^2 - q1^2 + q2^2 - q3^2;
   R_out(2, 3) = 2 * (q2 * q3 - q0 * q1);
   R_out(3, 1) = 2 * (q1 * q3 - q0 * q2);
   R_out(3, 2) = 2 * (q2 * q3 + q0 * q1);
   R_out(3, 3) = q0^2 - q1^2 - q2^2 + q3^2;

end
