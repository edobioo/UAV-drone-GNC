# Config script for ESKF implementation
# cfg -> config struct with ESKF parameters
# accel -> config struct of accelerometer
# gyroscope -> config struct of gyroscope
# gps -> config struct of gps
function [cfg] = eskf_config(accel, gyro, gps)

   # Constants
   cfg.g_ENU = [0; 0; -9.80665];

   # Frequencies
   assert(accel.sample_freq == gyro.sample_freq);
   cfg.prediction_freq = accel.sample_freq;
   cfg.prediction_time = 1 / cfg.prediction_freq;
   cfg.correction_freq = gps.sample_freq;

   # Process covariance (Q)
   dt = cfg.prediction_time;
   cfg.Q = kron(diag([accel.sigma_vrw^2 * dt^2, ...
                      gyro.sigma_arw^2 * dt^2, ...
                      accel.bias_K^2 * dt, ...
                      gyro.bias_K^2 * dt]), eye(3));

   # Measurement covariance (V)
   cfg.V = diag(gps.sigma_pos_ENU_vec.^2);

end
