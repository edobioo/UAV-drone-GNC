# Gyro measurement model
# omega_meas -> matrix 3xN with omega perturbed by measurement model
# omega_nom -> matrix 3xN with omega nominal
# gyro -> struct containing gyro parameters
function [omega_meas] = gyro_measurement(omega_nom, gyro)
    # Generate white noise block
    [M, N] = size(omega_nom);
    if (M != 3)
       error('Wrong row size of omega input vector')
    endif
    white_noise = gyro.sigma_arw * randn(3, N);

    # Generate bias noise block

    bias_noise = gyro.bias_K * sqrt(gyro.sample_time) * randn(3, N);
    # Cumulative sum of bias noise (rows)
    bias = cumsum(bias_noise, 2);

    # Generate measurement
    omega_meas = omega_nom + white_noise + bias;
end
