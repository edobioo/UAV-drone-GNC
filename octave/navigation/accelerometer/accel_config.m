# Accelerometer configuration script (ADIS16475)
function cfg = accel_config()

   # Sample frequency, sample time
   cfg.sample_freq = 200.0;
   cfg.sample_time = 1 / cfg.sample_freq;

   # Gravity
   cfg.g_ENU = [0; 0; -9.80665];

   # Datasheet values
   # Velocity Random Walk [(m/s)/sqrt(h)]
   cfg.vrw = 0.012;
   # Bias intability [micro-g]
   cfg.bias_instability = 3.6;

   # VRW deviation [m/s^2]
   cfg.sigma_vrw = (cfg.vrw / 60) * (1 / sqrt(cfg.sample_time));

   # Coefficient K bias random walk from Allan deviaton [(m/s²)/sqrt(s)]
   # See accel_compute_params.m
   cfg.bias_K = 8.485740283963149e-07;

end
