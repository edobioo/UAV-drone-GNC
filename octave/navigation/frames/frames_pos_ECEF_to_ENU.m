# Compute position in ENU frame from position in ECEF frame
# pos_ENU -> position in ENU frame
# pos_ECEF -> position in ECEF frame
# frames -> frames struct with transformation parameters
function [pos_ENU] = frames_pos_ECEF_to_ENU(pos_ECEF, frames)

   # First translate then apply transformation
   pos_ENU = frames_ECEF_to_ENU((pos_ECEF - frames.p_origin_ECEF), frames);

end
