function filename = generate_maneuver_trajectory()
   # Synthetic multirotor reference for testing the IMU/GNSS estimator.
   # Translation is prescribed analytically and thrust acts along body +Z.
   dt = 1 / 200;
   t = 0:dt:120;
   g = 9.80665;

   # Start in hover, climb to 14 m and smoothly enter/exit the maneuver.
   [climb, climb_dot, climb_ddot] = smooth_ramp(t, 6, 12);
   [enter, enter_dot, enter_ddot] = smooth_ramp(t, 18, 10);
   [exit_ramp, exit_dot, exit_ddot] = smooth_ramp(t, 104, 10);
   envelope = enter .* (1 - exit_ramp);
   envelope_dot = enter_dot .* (1 - exit_ramp) - enter .* exit_dot;
   envelope_ddot = enter_ddot .* (1 - exit_ramp) ...
                   - 2 * enter_dot .* exit_dot - enter .* exit_ddot;

   # A horizontal figure eight and a slower vertical oscillation.
   tau = t - 18;
   frequency = 2 * pi ./ [26; 13; 35];
   amplitude = [28; 12; 2.5];
   phase = frequency * tau;
   path = amplitude .* sin(phase);
   path_dot = (amplitude .* frequency) .* cos(phase);
   path_ddot = -(frequency .^ 2) .* path;
   pos_ENU = path .* envelope + [0; 0; 14] * climb;
   vel_ENU = path_dot .* envelope + path .* envelope_dot ...
             + [0; 0; 14] * climb_dot;
   acc_ENU = path_ddot .* envelope + 2 * path_dot .* envelope_dot ...
             + path .* envelope_ddot + [0; 0; 14] * climb_ddot;

   # Align body +Z with the specific force, choosing yaw independently.
   f_ENU = acc_ENU + [0; 0; g];
   thrust_per_mass = sqrt(sum(f_ENU .^ 2, 1));
   body_z_ENU = f_ENU ./ thrust_per_mass;
   yaw = envelope .* (1.05 * sin(2 * pi * tau / 19) ...
                        + 0.35 * sin(2 * pi * tau / 11));
   z_x = cos(yaw) .* body_z_ENU(1, :) + sin(yaw) .* body_z_ENU(2, :);
   z_y = -sin(yaw) .* body_z_ENU(1, :) + cos(yaw) .* body_z_ENU(2, :);
   z_z = body_z_ENU(3, :);
   roll = atan2(-z_y, hypot(z_x, z_z));
   pitch = atan2(z_x, z_z);

   # Hamilton scalar-first quaternion of Rz(yaw)*Ry(pitch)*Rx(roll).
   cr = cos(roll / 2);  sr = sin(roll / 2);
   cp = cos(pitch / 2); sp = sin(pitch / 2);
   cy = cos(yaw / 2);   sy = sin(yaw / 2);
   q = [cr .* cp .* cy + sr .* sp .* sy;
        sr .* cp .* cy - cr .* sp .* sy;
        cr .* sp .* cy + sr .* cp .* sy;
        cr .* cp .* sy - sr .* sp .* cy];

   # omega_body = 2 * vector(conj(q) * q_dot), using centered differences.
   # The first and last samples lie in stationary hover.
   q_dot = [zeros(4, 1), (q(:, 3:end) - q(:, 1:end-2)) / (2 * dt), zeros(4, 1)];
   s = q(1, :);
   u = q(2:4, :);
   u_dot = q_dot(2:4, :);
   omega_body = 2 * (s .* u_dot - q_dot(1, :) .* u - cross(u, u_dot, 1));
   f_body = [zeros(2, numel(t)); thrust_per_mass];

   filename = fullfile(fileparts(mfilename('fullpath')), 'output', 'uav1_maneuver_log.csv');
   fid = fopen(filename, 'w');
   if (fid < 0)
      error('Cannot open trajectory output: %s', filename);
   endif
   unwind_protect
      # Keep the legacy header used by the C++ simulator/viewer.
      fprintf(fid, ['Time,Position_ENU[0],Position_ENU[1],Position_ENU[2],' ...
                    'Velocity_ENU[0],Velocity_ENU[1],Velocity_ENU[2],' ...
                    'Quaternion_ENU_body[0],Quaternion_ENU_body[1],Quaternion_ENU_body[2],Quaternion_ENU_body[3],' ...
                    'Omega_body[0],Omega_body[1],Omega_body[2],' ...
                    'f_body[0],f_body[1],f_body[2]\n']);
      format = [repmat('%.17g,', 1, 16), '%.17g\n'];
      fprintf(fid, format, [t; pos_ENU; vel_ENU; q; omega_body; f_body]);
   unwind_protect_cleanup
      fclose(fid);
   end_unwind_protect
endfunction

function [s, s_dot, s_ddot] = smooth_ramp(t, start_time, duration)
   # This polynomial joins the constant segments with continuous jerk.
   u = min(1, max(0, (t - start_time) / duration));
   s = u .^ 4 .* (35 - 84 * u + 70 * u .^ 2 - 20 * u .^ 3);
   s_dot = 140 * u .^ 3 .* (1 - u) .^ 3 / duration;
   s_ddot = 420 * u .^ 2 .* (1 - u) .^ 2 .* (1 - 2 * u) / duration ^ 2;
endfunction
