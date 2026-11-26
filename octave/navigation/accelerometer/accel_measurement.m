# Accelerometer measurement model
# f_meas -> matrix 3xN with specific force perturbed by measurement model
# f_nom -> matrix 3xN with specific force nominal
# accel -> struct containing accelerometer parameters
function [f_meas] = accel_measurement(f_nom, accel)

    # Generate white noise block
    [M, N] = size(f_nom);
    if (M != 3)
       error('Wrong row size of f_nom input vector')
    endif
    white_noise = accel.sigma_vrw * randn(3, N);

    # Generate bias noise block
    bias_noise = accel.bias_K * sqrt(accel.sample_time) * randn(3, N);
    # Cumulative sum of bias noise (rows)
    bias = cumsum(bias_noise, 2);

    # Generate measurement
    f_meas = f_nom + white_noise + bias;
end
