# Conversion from ENU coordinates to ECEF coordinates
# vec_ECEF -> vector in ECEF coordinates
# vec_ENU -> vector in ENU coordinates
# frames -> frames struct with transformation parameters
function [vec_ECEF] = frames_ENU_to_ECEF(vec_ENU, frames)

   # Transformation ENU -> ECEF
   vec_ECEF = frames.R_ENU_ECEF * vec_ENU;

end
