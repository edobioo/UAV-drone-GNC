clear all, clc

# To compute gyro parameters from datasheet

# Constants
deg_to_rad = pi / 180;

# Timing
fs = 200;
ts = 1 / fs;

# ARW [deg/sqrt(h)]
arw = 0.15;
# Bias instability [deg/h]
bias_instability = 2;

# Conversion to SI
arw = (arw * deg_to_rad) / 60; # [rad/sqrt(s)]
bias_instability = bias_instability / 3600;

# Compute deviations for point 2 of Allan deviation graph
sigma_a = 5; # [deg/h]
sigma_a = (sigma_a * deg_to_rad) / 3600; # [rad/s]
tau = 10e3;
sigma_arw = arw / sqrt(tau);

# Find deviation for RRW bias
sigma_bias = sqrt(sigma_a^2 - sigma_arw^2);
format long
K = sigma_bias * sqrt(3 / tau)
