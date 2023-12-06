outputState = readtable("outputState.csv");
outputState = table2array(outputState);
time = readtable("imu_time.csv");
time = table2array(time);
mocap_pos = readtable("mocapPos_debug.csv");
mocap_pos = table2array(mocap_pos);
mocap_time = readtable("mocap_time.csv");
mocap_time = table2array(mocap_time);
mocap_range_min = find(abs(mocap_time - time(1)) == min(abs(mocap_time - time(1))));
mocap_range_max = find(abs(mocap_time - time(end)) == min(abs(mocap_time - time(end))));
mocap_range = mocap_range_min:mocap_range_max;

% rng = 1601:7600;
rng = 1:length(time);
time = time(rng);
estColor = 'b';
measColor = 'r';
lw = 1;

plot_position = true;
plot_attitude = true;
plot_velocity = false;
plot_biases = true;

% gt = [roll_imu(rng), -pitch_imu(rng), -yaw_imu(rng)];
% initIndex = find(time>20 & time<30);
% initBias = (mean(outputState(7:9,initIndex)))' - mean(gt(initIndex,:));


if plot_position
    figure()
    s1 = subplot(311);
    plot(time, outputState(1, :), Color=estColor, LineWidth=lw);
    hold on
    plot(mocap_time(mocap_range), mocap_pos(mocap_range, 1), Color=measColor, LineWidth=lw);
    hold off
    ylabel("X (m)")
    grid on;
    
    
    s2 = subplot(312);
    plot(time, outputState(2, :), Color=estColor, LineWidth=lw);
    hold on
    plot(mocap_time(mocap_range), mocap_pos(mocap_range, 2), Color=measColor, LineWidth=lw);
    hold off
    ylabel("Y (m)")
    grid on;
    
    s3 = subplot(313);
    plot(time, outputState(3, :), Color=estColor, LineWidth=lw);
    hold on
    plot(mocap_time(mocap_range), mocap_pos(mocap_range, 3), Color=measColor, LineWidth=lw);
    hold off
    ylabel("Z (m)")
    grid on;
    legend("Estimated position", "Measured position", Location="southeast")
    
    xlabel("time (s)");
    linkaxes([s1, s2, s3], 'xy')
end

% [t, x, y, z, q] = processMocapRigid("rc_circuits_mocap.csv", 30, -108.93);
% eul = quat2eul(q)*180/pi;

if plot_attitude
    figure()
    s1 = subplot(311);
    plot(time, outputState(7, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    if exist('roll_imu','var') == 1
        plot(time_imu(rng), roll_imu(rng), 'k.');
    end
    % plot(t, eul(:, 3), Color=measColor, LineStyle='-')
    hold off
    ylabel("\Phi (deg)")
    grid on;
    
    s2 = subplot(312);
    plot(time, outputState(8, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    if exist('pitch_imu','var') == 1
        plot(time_imu(rng), -pitch_imu(rng), 'k.');
    end
    % plot(t, eul(:, 2), Color=measColor, LineStyle='-')
    hold off
    ylabel("\Theta (deg)")
    grid on;
    
    s3 = subplot(313);
    plot(time, outputState(9, :)*180/pi, Color=estColor, LineWidth=lw);
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
    plot(time, outputState(4, :), Color=estColor, LineWidth=lw);
    ylabel("Vx (m/s)")
    grid on;
    
    subplot(312)
    plot(time, outputState(5, :), Color=estColor, LineWidth=lw);
    ylabel("Vy (m/s)")
    grid on;
    
    subplot(313)
    plot(time, outputState(6, :), Color=estColor, LineWidth=lw);
    ylabel("Vz (m/s)")
    grid on;
    
    xlabel("time (s)");
end

if plot_biases
    figure()
    subplot(231)
    plot(time, outputState(10, :), Color=estColor, LineWidth=lw);
    ylabel("")
    grid on;
    
    subplot(232)
    plot(time, outputState(11, :), Color=estColor, LineWidth=lw);
    ylabel("")
    grid on;
    
    subplot(233)
    plot(time, outputState(12, :), Color=estColor, LineWidth=lw);
    ylabel("")
    grid on;
    
    subplot(234)
    plot(time, outputState(13, :), Color=estColor, LineWidth=lw);
    ylabel("")
    grid on;
    
    subplot(235)
    plot(time, outputState(14, :), Color=estColor, LineWidth=lw);
    ylabel("")
    grid on;
    
    subplot(236)
    plot(time, outputState(15, :), Color=estColor, LineWidth=lw);
    ylabel("")
    grid on;
end