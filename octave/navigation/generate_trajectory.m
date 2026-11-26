clear all, clc, close all
# Generate nominal drone trajectory

addpath('math');

# Drone configuration parameters
uav.mass = 1.0;
uav.inertia = diag([0.00556, 0.00556, 0.01063]);
uav.inertia_inv = inv(uav.inertia);
uav.g_ENU = [0; 0; -9.80665];

# Geometry
M1.arm_to_COM = 0.19;
M1.pos_to_COM = [M1.arm_to_COM / sqrt(2); M1.arm_to_COM / sqrt(2); 0];
M2.arm_to_COM = 0.19;
M2.pos_to_COM = [-M1.arm_to_COM / sqrt(2); M1.arm_to_COM / sqrt(2); 0];
M3.arm_to_COM = 0.19;
M3.pos_to_COM = [-M1.arm_to_COM / sqrt(2); -M1.arm_to_COM / sqrt(2); 0];
M4.arm_to_COM = 0.19;
M4.pos_to_COM = [M1.arm_to_COM / sqrt(2); -M1.arm_to_COM / sqrt(2); 0];

# Thrust to body force matrix
uav.TB_F = zeros(3, 4);
uav.TB_F(end, :) = [1.0, 1.0, 1.0, 1.0];

# Thrust to body torque matrix
uav.TB_T = zeros(3, 4);
uav.TB_T(1, :) = [M1.pos_to_COM(2), M2.pos_to_COM(2), M3.pos_to_COM(2), M4.pos_to_COM(2)];
uav.TB_T(2, :) = [- M1.pos_to_COM(1), - M2.pos_to_COM(1), - M3.pos_to_COM(1), - M4.pos_to_COM(1)];

# Initial state
pos_ENU = [0; 0; 0];
vel_ENU = [0; 0; 0];
q_body_ENU = [1; 0; 0; 0];
omega_B = [0; 0; 0];
state = [pos_ENU; vel_ENU; q_body_ENU; omega_B];

# Load input trajectory
input_mat = csvread('./input/input_hover.csv', 1, 0);
thrust = input_mat(:, 2 : 5)';
# Timing
fs = 200;
ts = 1 / fs;
t_vec = input_mat(:, 1)';
n_samples = numel(t_vec);

# Loop
# State
state_log = zeros(size(state, 1), n_samples);
state_log(:, 1) = state;

# Specific body frame force for IMU
f_body = zeros(3, 1);
f_body_log = zeros(3, n_samples);
# Assign first log, supposing body starts aligned with ENU frame
f_body_log(:, 1) = - uav.g_ENU;
for i = 2 : n_samples
   [state, f_body] = integrate_state(uav, state, thrust(:, i), ts);

   state_log(:, i) = state;
   f_body_log(:, i) = f_body;
end

# Plot
figure(1), hold on
plot(t_vec, state_log(1, :))
plot(t_vec, state_log(2, :))
plot(t_vec, state_log(3, :))

# Write trajectory csv
filename = './output/uav1_log.csv';

# Write header
f = fopen(filename, 'w');
if (f == -1)
   error("Can't open output file");
end
# CSV header names follow the external simulator/viewer schema; see FRAME_CONVENTIONS.md.
fprintf(f, ['Time,Position_ENU[0],Position_ENU[1],Position_ENU[2],' ...
            'Velocity_ENU[0],Velocity_ENU[1],Velocity_ENU[2],' ...
            'Quaternion_ENU_body[0],Quaternion_ENU_body[1],Quaternion_ENU_body[2],Quaternion_ENU_body[3],' ...
            'Omega_body[0],Omega_body[1],Omega_body[2],' ...
            'f_body[0],f_body[1],f_body[2]\n']);
fclose(f);

# Print state log
dlmwrite(filename, [t_vec; state_log; f_body_log]', '-append', ...
         'delimiter', ',', 'precision', '%.17g');

