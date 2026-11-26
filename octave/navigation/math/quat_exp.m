# Compute generic quaternion exponential
# Scalar first convention q = [qw, qx, qy, qz]
# q_out -> output quaternion, scalar first
# q_in -> input quaternion, scalar first
function [q_out] = quat_exp(q_in)

   # Components
   qw = q_in(1);
   qx = q_in(2);
   qy = q_in(3);
   qz = q_in(4);

   # Vectorial part
   v = [qx, qy, qz];
   v_norm = sqrt(qx^2 + qy^2 + qz^2);

   # Fill out quaternion
   q_out = zeros(4, 1);
   q_out(1) = exp(qw) * cos(v_norm);
   if (v_norm != 0)
      q_out(2 : 4) = exp(qw) * ((v / v_norm) * sin(v_norm));
   else
      q_out(2 : 4 ) = [0; 0; 0];
   endif

end
