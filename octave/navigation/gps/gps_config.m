# GPS configuration script (ZED-F9P-05B)
# Using ENU coordinates in the model, the real position will be converted to
# ECEF coordinates afterwards
function [cfg] = gps_config()

   # Timing
   cfg.sample_freq = 5;
   cfg.sample_time = 1 / cfg.sample_freq;

   # Datasheet values (PVT)
   # Horizontal Position Accuracy CEP [m]
   cfg.horizontal_pos_accuracy_CEP = 1.5;

   # Vertical Position Accuracy median [m]
   cfg.vertical_pos_accuracy_median = 2.25;

   # Position error sigma [m] (local ENU frame)
   # Horizontal error -> Rayleigh distribution
   sigma_horizontal = cfg.horizontal_pos_accuracy_CEP / sqrt(log(4));
   cfg.sigma_pos_E = sigma_horizontal;
   cfg.sigma_pos_N = sigma_horizontal;
   # Vertical error -> Median
   cfg.sigma_pos_U = 1.4826 * cfg.vertical_pos_accuracy_median;
   # Sigma vector
   cfg.sigma_pos_ENU_vec = [cfg.sigma_pos_E; cfg.sigma_pos_N; cfg.sigma_pos_U];

   # Velocity error sigma [m/s] (TODO)

end
