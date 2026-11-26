# Prediction step of ESKF
# state_hat_out -> output nominal state predicted by ESKF
# P_out -> output error state covariance predicted by ESKF
# state_hat_in -> input nominal state predicted by ESKF
# P_in -> input error state covariance predicted by ESKF
# u_m -> input vector with accel and gyro measurements
# eskf -> config struct with ESKF parameters
function [state_hat_out, P_out] = eskf_prediction(state_hat_in, P_in, u_m, eskf)

   # Parameters
   dt = eskf.prediction_time;
   g_ENU = eskf.g_ENU;

   # Extract components from state vector
   p = state_hat_in(1 : 3);
   v = state_hat_in(4 : 6);
   q = state_hat_in(7 : 10);
   a_b = state_hat_in(11 : 13);
   w_b = state_hat_in(14 : 16);

   # Extract components from input vector
   a_m = u_m(1 : 3);
   w_m = u_m(4 : 6);

   # Rotation matrix
   R_body_ENU = quat_to_dcm(q);

   # Propagate nominal state
   p_new = p + v * dt + 0.5 * (R_body_ENU * (a_m - a_b) + g_ENU) * dt^2;
   v_new = v + (R_body_ENU * (a_m - a_b) + g_ENU) * dt;
   q_omega = [0; (w_m - w_b) * (dt / 2)];
   delta_q_new = quat_exp(q_omega);
   q_new = quat_norm(quat_prod(q, delta_q_new));
   a_b_new = a_b;
   w_b_new = w_b;

   # Compose state_hat_out
   state_hat_out = [p_new; v_new; q_new; a_b_new; w_b_new];

   # Propagate covariance P
   # Jacobian Fx
   # Small increment rotation matrix
   delta_R_body_ENU = quat_to_dcm(delta_q_new);
   # Skew symmetric matrix from acceleration
   a_skew_vec = (a_m - a_b) * dt;
   a_skew = [0, - a_skew_vec(3), a_skew_vec(2); ...
             a_skew_vec(3), 0, - a_skew_vec(1); ...
             - a_skew_vec(2), a_skew_vec(1), 0];
   # Compose Fx (15x15)
   Fx = [eye(3), eye(3) * dt, zeros(3, 3), zeros(3, 3), zeros(3, 3); ...
         zeros(3, 3), eye(3), - R_body_ENU * a_skew, - R_body_ENU * dt, zeros(3, 3); ...
         zeros(3, 3), zeros(3, 3), delta_R_body_ENU', zeros(3, 3), - eye(3) * dt; ...
         zeros(3, 3), zeros(3, 3), zeros(3, 3), eye(3), zeros(3, 3); ...
         zeros(3, 3), zeros(3, 3), zeros(3, 3), zeros(3, 3), eye(3)];

   # Compose Fi (15x12)
   Fi = [zeros(3, 12); eye(12)];

   # Update error state covariance P
   P_out = Fx * P_in * Fx' + Fi * eskf.Q * Fi';

end
