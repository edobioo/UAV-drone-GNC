# Apply correction to the ESKF prediction estimate
# state_hat_out -> output nominal state predicted by ESKF
# P_out -> output error state a posteriori covariance predicted by ESKF
# state_hat_in -> input nominal state predicted by ESKF
# P_in -> input error state covariance predicted by ESKF
# y_in -> measurement used to correct estimate
# eskf -> config struct with ESKF parameters
function [state_hat_out, P_out] = eskf_correction(state_hat_in, P_in, y_in, eskf)

   # Hx -> Jacobian of measurement wrt true state (3x16)
   Hx = [eye(3), zeros(3, 13)];

   # Xdx -> Jacobian of true state wrt error state (16x15)
   # Extract quaternion from predicted state (linearization point)
   q_body_ENU = state_hat_in(7 : 10);
   qw = q_body_ENU(1);
   qx = q_body_ENU(2);
   qy= q_body_ENU(3);
   qz = q_body_ENU(4);
   Qdx = 0.5 * [-qx, -qy, -qz; ...
                qw, -qz, qy; ...
                qz, qw, -qx; ...
                -qy, qx, qw];
   Xdx = blkdiag(eye(6), Qdx, eye(6));

   # Compute Jacobian H (3x15)
   H = Hx * Xdx;

   # Compute Kalman Gain K
   K = P_in * H' * inv(H * P_in * H' + eskf.V);

   # Compute estimated error state
   dx_hat = K * (y_in - state_hat_in(1 : 3));

   # Compute estimated state
   state_hat_out = zeros(16, 1);
   state_hat_out(1 : 6) = state_hat_in(1 : 6) + dx_hat(1 : 6);
   state_hat_out(7 : 10) = quat_norm(quat_prod(state_hat_in(7 : 10), ...
                                               quat_exp([0; dx_hat(7 : 9) / 2])));
   state_hat_out(11 : end) = state_hat_in(11 : end) + dx_hat(10 : end);

   # Compute a posteriori covariance P (symmetric)
   P_out = P_in - K * (H * P_in * H' + eskf.V) * K';

   # Compute Jacobian G
   dtheta_skew_vec = dx_hat(7 : 9);
   dtheta_skew = [0, - dtheta_skew_vec(3), dtheta_skew_vec(2); ...
                  dtheta_skew_vec(3), 0, - dtheta_skew_vec(1); ...
                  - dtheta_skew_vec(2), dtheta_skew_vec(1), 0];
   dtheta_blk = eye(3) - 0.5 * dtheta_skew;
   G = blkdiag(eye(6), dtheta_blk, eye(6));

   # Reset a posteriori covariance P
   P_out = G * P_out * G';

end
