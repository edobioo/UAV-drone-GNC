# Quaternion Hamilton product, scalar first
# q_out -> output quaternion
# p_in -> first input quaternion of the product
# q_in -> second input quaternion of the product

function [q_out] = quat_prod(p_in, q_in)

   # Using matrix form [p_in]_L * q_in, Solà
   pw = p_in(1);
   px = p_in(2);
   py = p_in(3);
   pz = p_in(4);

   p_in_matL = [pw, -px, -py, -pz; ...
                px, pw, -pz, py; ...
                py, pz, pw, -px; ...
                pz, -py, px, pw];

   q_out = p_in_matL * q_in;

end
