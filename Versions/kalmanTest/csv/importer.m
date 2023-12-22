clear;
close all;

outputState = readtable("outputState.csv");
outputState = table2array(outputState);
time = readtable("imu_time.csv");
time = table2array(time);
mocap_pos = readtable("mocapPos_debug.csv");
mocap_pos = table2array(mocap_pos);
mocap_time = readtable("mocap_time.csv");
mocap_time = table2array(mocap_time);

% Fix any non-unique points in the mocap times
[mocap_time, iu] = unique(mocap_time);
mocap_pos = mocap_pos(iu, :);

minTime = 0;
maxTime = 350;
mocap_idx = find(mocap_time >= minTime & mocap_time <= maxTime);
est_idx = find(time >= minTime & time <= maxTime);

estColor = 'b';
measColor = 'r';
lw = 1;

plot_position = true;
plot_attitude = true;
plot_velocity = false;
plot_biases = true;
plot_posError = true;

time_trunc = time(est_idx);
output_trunc = outputState(:, est_idx);
mocap_time_trunc = mocap_time(mocap_idx);
mocap_pos_trunc = mocap_pos(mocap_idx, :);

% Interpolate mocap_pos to align it with estimates
mocap_pos_trunc_xi = interp1(mocap_time_trunc, mocap_pos_trunc(:,1), time_trunc);
mocap_pos_trunc_yi = interp1(mocap_time_trunc, mocap_pos_trunc(:,2), time_trunc);
mocap_pos_trunc_zi = interp1(mocap_time_trunc, mocap_pos_trunc(:,3), time_trunc);

errorX = mocap_pos_trunc_xi - output_trunc(1, :)';
errorY = mocap_pos_trunc_yi - output_trunc(2, :)';
errorZ = mocap_pos_trunc_zi - output_trunc(3, :)';

rmseX = rms(errorX, "omitnan");
rmseY = rms(errorY, "omitnan");
rmseZ = rms(errorZ, "omitnan");


if plot_position
    figure()
    s1 = subplot(311);
    plot(time_trunc, output_trunc(1, :), Color=estColor, LineWidth=lw);
    hold on
    plot(mocap_time_trunc, mocap_pos_trunc(:, 1), Color=measColor, LineWidth=lw);
    hold off
    ylabel("X (m)")
    grid on;
    
    
    s2 = subplot(312);
    plot(time_trunc, output_trunc(2, :), Color=estColor, LineWidth=lw);
    hold on
    plot(mocap_time_trunc, mocap_pos_trunc(:, 2), Color=measColor, LineWidth=lw);
    hold off
    ylabel("Y (m)")
    grid on;
    
    s3 = subplot(313);
    plot(time_trunc, output_trunc(3, :), Color=estColor, LineWidth=lw);
    hold on
    plot(mocap_time_trunc, mocap_pos_trunc(:, 3), Color=measColor, LineWidth=lw);
    hold off
    ylabel("Z (m)")
    grid on;
    legend("Estimated position", "Measured position", Location="southeast")
    
    xlabel("time (s)");
    linkaxes([s1, s2, s3], 'x')
end

if plot_attitude
    figure()
    s1 = subplot(311);
    plot(time_trunc, output_trunc(7, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    if exist('roll_imu','var') == 1
        plot(time_imu(rng), roll_imu(rng), 'k.');
    end

    % plot(t, eul(:, 3), Color=measColor, LineStyle='-')
    hold off
    ylabel("\Phi (deg)")
    grid on;
    
    s2 = subplot(312);
    plot(time_trunc, output_trunc(8, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    if exist('pitch_imu','var') == 1
        plot(time_imu(rng), -pitch_imu(rng), 'k.');
    end
    % plot(t, eul(:, 2), Color=measColor, LineStyle='-')
    hold off
    ylabel("\Theta (deg)")
    grid on;
    
    s3 = subplot(313);
    plot(time_trunc, output_trunc(9, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    if exist('yaw_imu','var') == 1
        plot(time_imu(rng), -yaw_imu(rng), 'k.-');
    end
    % plot(t, eul(:, 1), Color=measColor, LineStyle='-')
    hold off
    ylabel("\Psi (deg)")
    grid on;
    
    xlabel("time (s)");
    linkaxes([s1, s2, s3], 'x')

end


if plot_velocity
    figure()
    subplot(311)
    plot(time_trunc, output_trunc(4, :), Color=estColor, LineWidth=lw);
    ylabel("Vx (m/s)")
    grid on;
    
    subplot(312)
    plot(time_trunc, output_trunc(5, :), Color=estColor, LineWidth=lw);
    ylabel("Vy (m/s)")
    grid on;
    
    subplot(313)
    plot(time_trunc, output_trunc(6, :), Color=estColor, LineWidth=lw);
    ylabel("Vz (m/s)")
    grid on;
    
    xlabel("time (s)");
end

if plot_biases
    figure()
    subplot(311)
    plot(time_trunc, output_trunc(10, :), Color=estColor, LineWidth=lw);
    title("Acc Biases")
    ylabel("")
    grid on;
    
    subplot(312)
    plot(time_trunc, output_trunc(11, :), Color=estColor, LineWidth=lw);
    ylabel("")
    grid on;
    
    subplot(313)
    plot(time_trunc, output_trunc(12, :), Color=estColor, LineWidth=lw);
    ylabel("")
    grid on;
    
    figure()
    subplot(311)
    plot(time_trunc, output_trunc(13, :), Color=estColor, LineWidth=lw);
    title("Gyro Biases")
    ylabel("")
    grid on;
    
    subplot(312)
    plot(time_trunc, output_trunc(14, :), Color=estColor, LineWidth=lw);
    ylabel("")
    grid on;
    
    subplot(313)
    plot(time_trunc, output_trunc(15, :), Color=estColor, LineWidth=lw);
    ylabel("")
    grid on;
end

if plot_posError
    figure()
    s1 = subplot(311);
    plot(time_trunc, errorX, Color=estColor, LineWidth=lw);
    ylabel("X Error (m)")
    grid on;
    
    s2 = subplot(312);
    plot(time_trunc, errorY, Color=estColor, LineWidth=lw);
    ylabel("Y Error (m)")
    grid on;
    
    s3 = subplot(313);
    plot(time_trunc, errorZ, Color=estColor, LineWidth=lw);
    ylabel("Z Error (m)")
    grid on;
    
    xlabel("time (s)");
    linkaxes([s1, s2, s3], 'x')
end
