# Gyro configuration script (ADIS16475)
function cfg = gyro_config()

   # Constants
   deg_to_rad = pi / 180;

   # Sample frequency, sample time
   cfg.sample_freq = 200;
   cfg.sample_time = 1 / cfg.sample_freq;

   # Datasheet values
   # Angle Random Walk [deg/sqrt(h)]
   cfg.arw = 0.15;
   # Bias instability [deg/h]
   cfg.bias_instability = 2;

   # SI units values
   # ARW deviation [rad/s]
   cfg.sigma_arw = (1 / 60) * ((cfg.arw * deg_to_rad) / sqrt(cfg.sample_time));

   # Coefficient K bias random walk from Allan deviaton [(rad/s)/sqrt(s))]
   # See gyro_compute_params.m
   cfg.bias_K = 4.197929409566389e-07;

end
