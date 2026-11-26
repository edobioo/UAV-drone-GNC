# Compute estimated position in ECEF coordinates from GPS
# pos_meas_ECEF -> output measured position in ECEF
# pos_nom_ENU -> input nominal position in ENU
# gps -> cfg struct with gps parameters
# frames -> cfg struct with frames parameters
function [pos_meas_ECEF] = gps_measurement(pos_nom_ENU, gps, frames)

    # Generate white noise block
    [M, N] = size(pos_nom_ENU);
    if (M != 3)
       error('Wrong row size of pos_nom_ENU input vector')
    endif
    white_noise = gps.sigma_pos_ENU_vec .* randn(3, N);

    # Generate measurement (ENU)
    pos_meas_ENU = pos_nom_ENU + white_noise;

    # Generate measurement (ECEF)
    pos_meas_ECEF = frames_pos_ENU_to_ECEF(pos_meas_ENU, frames);

end
