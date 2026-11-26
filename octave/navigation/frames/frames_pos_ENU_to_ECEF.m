# Compute position in ECEF frame from position in ENU frame
# pos_ECEF -> position in ECEF frame
# pos_ENU -> position in ENU frame
# frames -> frames struct with transformation parameters
function [pos_ECEF] = frames_pos_ENU_to_ECEF(pos_ENU, frames)

   # First apply transformation then translate
   pos_ECEF = frames_ENU_to_ECEF(pos_ENU, frames) + frames.p_origin_ECEF;

end
