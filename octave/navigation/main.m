close all, clear, clc

%% Init
# Add dirs to path
addpath('gyro');
addpath('accelerometer');
addpath('gps');
addpath('eskf');
addpath('math');
addpath('frames');

# Seed simulation
seed = 1999;
rng(seed);

# Load config
gyro = gyro_config();
accel = accel_config();
frames = frames_config();
gps = gps_config();
eskf = eskf_config(accel, gyro, gps);

# Load trajectory
input_data = csvread('./output/uav1_maneuver_log.csv', 1, 0);

# Timing
fs = 200;
ts = 1 / fs;
t_vec = input_data(:, 1);
n_samples = size(t_vec, 1);

# Nominal position
pos_nom_ENU = input_data(:, 2 : 4)';

# Nominal omega
omega_nom = input_data(:, 12 : 14)';

# Nominal specific force
f_nom = input_data(:, 15 : end)';

%% Generate measurements
# Measured omega from gyro
omega_meas = gyro_measurement(omega_nom, gyro);

# Measured force from accelerometer
f_meas = accel_measurement(f_nom, accel);

# Position measurement from gps
pos_meas_ECEF_GPS = gps_measurement(pos_nom_ENU, gps, frames);
# Convert ECEF GPS position to ENU position
pos_meas_ENU_GPS = frames_pos_ECEF_to_ENU(pos_meas_ECEF_GPS, frames);


%% Initial state
p0 = input_data(1, 2 : 4)';
v0 = input_data(1, 5 : 7)';
q_body_ENU = accel_to_attitude(f_meas(:, 1), accel);

%% Initial ESKF state
# Predicted nominal state
p_pred = zeros(3, 1);
v_pred = zeros(3, 1);
q_pred = [1; 0; 0; 0];
a_b_pred = zeros(3, 1);
w_b_pred = zeros(3, 1);
state_pred = [p_pred; v_pred; q_pred; a_b_pred; w_b_pred];
# Error state covariance
P = zeros(15, 15);

%% Measurement vector
y = pos_meas_ENU_GPS(:, 1);

%% Simulation loop
# Create log matrices
p_meas_ENU_IMU = zeros(3, n_samples);
v_meas_ENU_IMU = zeros(3, n_samples);
q_meas = zeros(4, n_samples);
state_eskf_log = zeros(16, n_samples);


# Init first log
p_meas_ENU_IMU(:, 1) = p0;
v_meas_ENU_IMU(:, 1) = v0;
q_meas(:, 1) = q_body_ENU;
state_eskf_log(:, 1) = state_pred;

tic

# Loop (200 Hz)
# Counters
counter_5_Hz = 1; # to synchronize with for loop
dec_200_5_Hz = eskf.prediction_freq / eskf.correction_freq;
for i = 2 : n_samples
   # Integration
   q_body_ENU = gyro_kinematics(q_body_ENU, omega_meas(:, i), ts);
   [p0, v0] = accel_kinematics(f_meas(:, i), q_body_ENU, p0, v0, accel);

   # Navigation
   # Prediction
   u_m = [f_meas(:, i); omega_meas(:, i)];
   [state_pred, P] = eskf_prediction(state_pred, P, u_m, eskf);
   counter_5_Hz++;

   # Correction
   if (counter_5_Hz == (dec_200_5_Hz + 1))
      y = pos_meas_ENU_GPS(:, i);
      [state_pred, P] = eskf_correction(state_pred, P, y, eskf);
      # Reset counter
      counter_5_Hz = 1;
   endif

   # Log data
   p_meas_ENU_IMU(:, i) = p0;
   v_meas_ENU_IMU(:, i) = v0;
   q_meas(:, i) = q_body_ENU;
   state_eskf_log(:, i) = state_pred;
endfor

toc

%% Plots
figure(1, 'name', 'ENU position', 'numbertitle', 'off');
subplot(2, 2, 1);
plot(t_vec, pos_nom_ENU.');
title('Reference position');
ylabel('Position [m]');
legend('East', 'North', 'Up');
grid on;

subplot(2, 2, 2);
plot(t_vec, p_meas_ENU_IMU.');
title('IMU position');
ylabel('Position [m]');
legend('East', 'North', 'Up');
grid on;

subplot(2, 2, 3);
gps_fix_idx = (1 + dec_200_5_Hz) : dec_200_5_Hz : n_samples;
plot(t_vec(gps_fix_idx), pos_meas_ENU_GPS(:, gps_fix_idx).');
title('GPS position used by ESKF');
xlabel('Time [s]');
ylabel('Position [m]');
legend('East', 'North', 'Up');
grid on;

subplot(2, 2, 4);
plot(t_vec, state_eskf_log(1 : 3, :).');
title('ESKF position');
xlabel('Time [s]');
ylabel('Position [m]');
legend('East', 'North', 'Up');
grid on;

figure(2, 'name', 'ENU velocity', 'numbertitle', 'off');
subplot(3, 1, 1);
plot(t_vec, input_data(:, 5 : 7));
title('Reference velocity');
ylabel('Velocity [m/s]');
legend('East', 'North', 'Up');
grid on;

subplot(3, 1, 2);
plot(t_vec, v_meas_ENU_IMU.');
title('IMU velocity');
ylabel('Velocity [m/s]');
legend('East', 'North', 'Up');
grid on;

subplot(3, 1, 3);
plot(t_vec, state_eskf_log(4 : 6, :).');
title('ESKF velocity');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('East', 'North', 'Up');
grid on;

figure(3, 'name', 'Attitude and angular rate', 'numbertitle', 'off');
subplot(2, 2, 1);
plot(t_vec, input_data(:, 8 : 11));
title('Reference quaternion');
legend('w', 'x', 'y', 'z');
grid on;

subplot(2, 2, 2);
plot(t_vec, q_meas.');
title('IMU quaternion');
legend('w', 'x', 'y', 'z');
grid on;

subplot(2, 2, 3);
plot(t_vec, state_eskf_log(7 : 10, :).');
title('ESKF quaternion');
xlabel('Time [s]');
legend('w', 'x', 'y', 'z');
grid on;

subplot(2, 2, 4);
plot(t_vec, omega_meas.');
title('Gyroscope angular rate (body)');
xlabel('Time [s]');
ylabel('Angular rate [rad/s]');
legend('x', 'y', 'z');
grid on;
