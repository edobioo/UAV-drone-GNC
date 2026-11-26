# Quaternion normalization
# q_out -> output quaternion
# q_in -> input quaternion
function [q_out] = quat_norm(q_in)
   q_in_norm = sqrt(q_in(1)^2 + q_in(2)^2 + q_in(3)^2 + q_in(4)^2);
   q_out = q_in / q_in_norm;
end
