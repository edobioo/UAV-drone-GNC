# Cross product between 3x1 vectors
# v_out -> output vector
# a_in -> first input vector
# b_in -> second input vector
function [v_out] = vec3_cross(a_in, b_in)
   v_out(1, 1) = (a_in(2) * b_in(3)) - (a_in(3) * b_in(2));
   v_out(2, 1) = (a_in(3) * b_in(1)) - (a_in(1) * b_in(3));
   v_out(3, 1) = (a_in(1) * b_in(2)) - (a_in(2) * b_in(1));
end
