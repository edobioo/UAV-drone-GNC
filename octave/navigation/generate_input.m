clear all, clc, close all

# Generate input sequence
# Drone parameters
mass = 1.0;
max_thrust = 5.0;
# Gravity vector
g_ENU = [0; 0; -9.80665];
# Timing
fs = 200.0;
ts = 1 / fs;

# ---------------------------------------------------------------------------- #
# Vertical ascent + hover
# Max acceleration
a_ENU_max = [0; 0; 1]; # vertical component
a_ENU_max = a_ENU_max * ((4 * max_thrust) / mass) + g_ENU;

# Considering a_ENU_max / 6
a_ENU_const = a_ENU_max / 6;

# Build linear vertical velocity profile
# Hover phase
thrust_hover_const = - (mass * g_ENU(3)) / 4;

# Block 1 -> constant acceleration ascent
delta_t_block1 = 10.0; # first ascent phase duration, linear velocity profile
t_start = 0.0;
t_vec_block1 = t_start : ts : delta_t_block1;
v_ENU_block1 = a_ENU_const * t_vec_block1;
# Thrust from chosen acceleration
thrust_const = (mass / 4) * (a_ENU_const(3) - g_ENU(3));
thrust_block1 = thrust_const * ones(size(t_vec_block1));

# Block 2 -> deceleration to hover
delta_t_block2 = 5.0; # impose deceleration time
t_vec_block2 = t_start : ts : delta_t_block2;
d_ENU_const = - v_ENU_block1(:, end) / delta_t_block2;
thrust_block2 = (mass / 4) * (d_ENU_const(3) - g_ENU(3)) * ones(size(t_vec_block2));

# Block 3 -> hover
delta_t_block3 = 500.0;
t_vec_block3 = t_start : ts : delta_t_block3;
thrust_block3 = thrust_hover_const * ones(size(t_vec_block3));

# Build complete trajectory
t_tot = delta_t_block1 + delta_t_block2 + delta_t_block3;
t_vec = 0.0 : ts : t_tot;
thrust_vec = [thrust_block1, thrust_block2(2 : end), thrust_block3(2 : end)];

# Build input trajectory matrix
thrust_mat = [thrust_vec; thrust_vec; thrust_vec; thrust_vec];

# Write csv
# Write trajectory csv
filename = './input/input_hover.csv';

# Write header
f = fopen(filename, 'w');
if (f == -1)
   error("Can't open output file");
end
fprintf(f, ['Time,T1,T2,T3,T4\n']);
fclose(f);

# Print state log
dlmwrite(filename, [t_vec; thrust_mat]', '-append', ...
         'delimiter', ',', 'precision', '%.17g');


