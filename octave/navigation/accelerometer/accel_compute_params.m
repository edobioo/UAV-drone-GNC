clear all, clc

# To compute accelerometer parameters from datasheet

# Gravity [m/s^2]
g = 9.80665;

# Timing
fs = 200;
ts = 1 / fs;

# VRW [m/(s*sqrt(h))]
vrw = 0.012;
# Bias instability [micro-g]
bias_instability = 3.6;

# Conversion to SI
vrw = vrw / 60; # [m/(s*sqrt(s))]
bias_instability = bias_instability * 1e-6 * g;

# Compute deviations for point 2 of Allan deviation graph
sigma_a = 5 * 1e-6 * g;
tau = 10e3;
sigma_vrw = vrw / sqrt(tau);

# Find deviation for RRW bias
sigma_bias = sqrt(sigma_a^2 - sigma_vrw^2);
format long
K = sigma_bias * sqrt(3 / tau)
