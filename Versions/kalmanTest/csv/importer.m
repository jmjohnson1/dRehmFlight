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

estColor = 'b';
measColor = 'r';
lw = 0.5;

plot_position = true;
plot_attitude = true;
plot_velocity = true;
plot_biases = true;

if plot_position
    figure()
    s1 = subplot(311);
    plot(time, outputState(1, :), Color=estColor, LineWidth=lw);
    hold on
    plot(mocap_time(mocap_range), mocap_pos(mocap_range, 1), Color=measColor, LineWidth=lw);
    hold off
    ylabel("X (m)")
    
    s2 = subplot(312);
    plot(time, outputState(2, :), Color=estColor, LineWidth=lw);
    hold on
    plot(mocap_time(mocap_range), mocap_pos(mocap_range, 2), Color=measColor, LineWidth=lw);
    hold off
    ylabel("Y (m)")
    
    s3 = subplot(313);
    plot(time, outputState(3, :), Color=estColor, LineWidth=lw);
    hold on
    plot(mocap_time(mocap_range), mocap_pos(mocap_range, 3), Color=measColor, LineWidth=lw);
    hold off
    ylabel("Z (m)")
    
    xlabel("time (s)");
    linkaxes([s1, s2, s3], 'xy')
end

% [t, x, y, z, q] = processMocapRigid("rc_circuits_mocap.csv", 30, -108.93);
% eul = quat2eul(q)*180/pi;

if plot_attitude
    figure()
    subplot(311)
    plot(time, outputState(7, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    if exist('roll_imu','var') == 1
        plot(time, -roll_imu(rng), 'k.');
    end
    % plot(t, eul(:, 3), Color=measColor, LineStyle='-')
    hold off
    ylabel("\Phi (deg)")
    
    subplot(312)
    plot(time, outputState(8, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    if exist('pitch_imu','var') == 1
        plot(time, -pitch_imu(rng), 'k.');
    end
    % plot(t, eul(:, 2), Color=measColor, LineStyle='-')
    hold off
    ylabel("\Theta (deg)")
    
    subplot(313)
    plot(time, outputState(9, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    if exist('yaw_imu','var') == 1
        plot(time, -yaw_imu(rng), 'k.-');
    end
    % plot(t, eul(:, 1), Color=measColor, LineStyle='-')
    hold off
    ylabel("\Psi (deg)")
    
    xlabel("time (s)");
end

% Velocity estimate using first order finite difference
vx = FDA_fourPoint(mocap_pos(:,1), 1/30);
vy = FDA_fourPoint(mocap_pos(:,2), 1/30);
vz = FDA_fourPoint(mocap_pos(:,3), 1/30);
velocity_fda = [0 0 0; vx' vy' vz'];
checkFda = true;
if plot_velocity
    figure()
    subplot(311)
    plot(time, outputState(4, :), Color=estColor, LineWidth=lw);
    if checkFda
        hold on
        plot(mocap_time(mocap_range), velocity_fda(mocap_range,1), color=measColor, LineWidth=lw);
        hold off
    end
    
    ylabel("Vx (m/s)")
    
    subplot(312)
    plot(time, outputState(5, :), Color=estColor, LineWidth=lw);
    if checkFda
        hold on
        plot(mocap_time(mocap_range), velocity_fda(mocap_range,2), color=measColor, LineWidth=lw);
        hold off
    end
    ylabel("Vy (m/s)")
    
    subplot(313)
    plot(time, outputState(6, :), Color=estColor, LineWidth=lw);
    if checkFda
        hold on
        plot(mocap_time(mocap_range), velocity_fda(mocap_range,3), color=measColor, LineWidth=lw);
        hold off
    end
    ylabel("Vz (m/s)")
    
    xlabel("time (s)");
end

if plot_biases
    figure()
    subplot(231)
    plot(time, outputState(10, :), Color=estColor, LineWidth=lw);
    ylabel("")
    
    subplot(232)
    plot(time, outputState(11, :), Color=estColor, LineWidth=lw);
    ylabel("")
    
    subplot(233)
    plot(time, outputState(12, :), Color=estColor, LineWidth=lw);
    ylabel("")
    
    subplot(234)
    plot(time, outputState(13, :), Color=estColor, LineWidth=lw);
    ylabel("")
    
    subplot(235)
    plot(time, outputState(14, :), Color=estColor, LineWidth=lw);
    ylabel("")
    
    subplot(236)
    plot(time, outputState(15, :), Color=estColor, LineWidth=lw);
    ylabel("")
end