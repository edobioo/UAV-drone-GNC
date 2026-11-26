# State integration using Forward Euler
# Nominal accelerometer measurement generation
# state_out -> output state vector
# f_body_out -> output nominal specific force in body frame
# uav -> drone params struct
# state_in -> input state vector
# input -> input vector (thrusts)
# dt -> integration step
function [state_out, f_body_out] = integrate_state(uav, state_in, input, dt)

   # Extract state components
   pos = state_in(1 : 3);
   vel = state_in(4 : 6);
   q = state_in(7 : 10);
   q0 = q(1);
   q1 = q(2);
   q2 = q(3);
   q3 = q(4);
   omega = state_in(11 : 13);

   # Body-to-ENU DCM: v_ENU = R_body_ENU * v_body
   R_body_ENU = [q0^2 + q1^2 - q2^2 - q3^2, 2*(q1 * q2 - q0 * q3), 2*(q1 * q3 + q0 * q2); ...
              2*(q1 * q2 + q0 * q3), q0^2 - q1^2 + q2^2 - q3^2, 2*(q2 * q3 - q0 * q1); ...
              2*(q1 * q3 - q0 * q2), 2*(q2 * q3 + q0 * q1), q0^2 - q1^2 - q2^2 + q3^2];

   # State dot
   pos_dot = vel;
   vel_dot = (1 / uav.mass) * R_body_ENU * uav.TB_F * input + uav.g_ENU;
   q_dot = 0.5 * quat_prod(q, [0; omega]);
   omega_dot = uav.inertia_inv * (uav.TB_T * input - vec3_cross(omega, (uav.inertia * omega)));

   # Integrate
   state_dot = [pos_dot; vel_dot; q_dot; omega_dot];

   # Compute updated state
   state_out = state_in + state_dot * dt;

   # Compute specific force for IMU model
   f_body_out = (uav.TB_F * input) / uav.mass;

end
