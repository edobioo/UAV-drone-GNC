# Conversion from ECEF coordinates to ENU coordinates
# vec_ENU -> vector in ENU coordinates
# vec_ECEF -> vector in ECEF coordinates
# frames -> frames struct with transformation parameters
function [vec_ENU] = frames_ECEF_to_ENU(vec_ECEF, frames)

   # Transformation ECEF -> ENU
   vec_ENU = frames.R_ENU_ECEF.' * vec_ECEF;

end
