# Configuration script for transformations between ENU and ECEF frames
# frames -> output struct containing ENU origin position and rotation matrix
function [cfg] = frames_config()

   # Constants
   deg_to_rad = pi / 180;

   # Position of the origin of ENU frame  in ECEF (lat, lon, radius)
   cfg.lat_origin_ENU = 45.0867; # [deg]
   cfg.lon_origin_ENU = 7.6389; # [deg]
   cfg.radius_origin_ENU = 6367.712e3; # [m]

   # Position of the origin of ENU frame in ECEF (x, y, z)
   x_origin_ECEF = cfg.radius_origin_ENU * ...
                   cos(cfg.lat_origin_ENU * deg_to_rad) * ...
                   cos(cfg.lon_origin_ENU * deg_to_rad);

   y_origin_ECEF = cfg.radius_origin_ENU * ...
                   cos(cfg.lat_origin_ENU * deg_to_rad) * ...
                   sin(cfg.lon_origin_ENU * deg_to_rad);

   z_origin_ECEF = cfg.radius_origin_ENU * ...
                   sin(cfg.lat_origin_ENU * deg_to_rad);

   cfg.p_origin_ECEF = [x_origin_ECEF; y_origin_ECEF; z_origin_ECEF];

   # Passive rotation matrix ENU to ECEF
   phi = cfg.lat_origin_ENU * deg_to_rad;
   lambda = cfg.lon_origin_ENU * deg_to_rad;
   cfg.R_ENU_ECEF = zeros(3, 3);
   cfg.R_ENU_ECEF(1, 1) = - sin(lambda);
   cfg.R_ENU_ECEF(1, 2) = - cos(lambda) * sin(phi);
   cfg.R_ENU_ECEF(1, 3) = cos(lambda) * cos(phi);
   cfg.R_ENU_ECEF(2, 1) = cos(lambda);
   cfg.R_ENU_ECEF(2, 2) = - sin(lambda) * sin(phi);
   cfg.R_ENU_ECEF(2, 3) = sin(lambda) * cos(phi);
   cfg.R_ENU_ECEF(3, 1) = 0;
   cfg.R_ENU_ECEF(3, 2) = cos(phi);
   cfg.R_ENU_ECEF(3, 3) = sin(phi);

end
