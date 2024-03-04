clear;
close all;

outputState = readtable("outputState.csv");
outputState = table2array(outputState);

load("mocapData.mat");
mocap_pos = mocapPosition_i;


minTime = 0;
maxTime = 1000;
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
mocap_pos_trunc = mocap_pos(est_idx, :);


errorX = mocap_pos_trunc(:, 1) - output_trunc(1, :)';
errorY = mocap_pos_trunc(:, 2) - output_trunc(2, :)';
errorZ = mocap_pos_trunc(:, 3) - output_trunc(3, :)';

rmseX = rms(errorX, "omitnan");
rmseY = rms(errorY, "omitnan");
rmseZ = rms(errorZ, "omitnan");


if plot_position
    figure()
    s1 = subplot(311);
    plot(time_trunc, output_trunc(1, :), Color=estColor, LineWidth=lw);
    hold on
    plot(time_trunc, mocap_pos_trunc(:, 1), Color=measColor, LineWidth=lw);
    hold off
    ylabel("X (m)")
    grid on;
    
    
    s2 = subplot(312);
    plot(time_trunc, output_trunc(2, :), Color=estColor, LineWidth=lw);
    hold on
    plot(time_trunc, mocap_pos_trunc(:, 2), Color=measColor, LineWidth=lw);
    hold off
    ylabel("Y (m)")
    grid on;
    
    s3 = subplot(313);
    plot(time_trunc, output_trunc(3, :), Color=estColor, LineWidth=lw);
    hold on
    plot(time_trunc, mocap_pos_trunc(:, 3), Color=measColor, LineWidth=lw);
    hold off
    ylabel("Z (m)")
    grid on;
    legend("Estimated position", "Measured position", Location="southeast")
    
    xlabel("time (s)");
    linkaxes([s1, s2, s3], 'x')
end

if plot_attitude
    eul = quat2eul(q_i(est_idx, :))*180/pi;
    figure()
    s1 = subplot(311);
    plot(time_trunc, output_trunc(7, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    if exist('roll_imu','var') == 1
        plot(time_imu(rng), roll_imu(rng), 'k.');
    end
    plot(time_trunc, eul(:, 3), Color=measColor, LineStyle='-')
    hold off
    ylabel("\Phi (deg)")
    grid on;
    
    s2 = subplot(312);
    plot(time_trunc, output_trunc(8, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    if exist('pitch_imu','var') == 1
        plot(time_imu(rng), -pitch_imu(rng), 'k.');
    end
    plot(time_trunc, eul(:, 2), Color=measColor, LineStyle='-')
    hold off
    ylabel("\Theta (deg)")
    grid on;
    
    s3 = subplot(313);
    plot(time_trunc, output_trunc(9, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    if exist('yaw_imu','var') == 1
        plot(time_imu(rng), -yaw_imu(rng), 'k.-');
    end
    plot(time_trunc, eul(:, 1), Color=measColor, LineStyle='-')
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
