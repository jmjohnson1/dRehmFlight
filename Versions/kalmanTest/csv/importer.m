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
    subplot(311)
    plot(time, outputState(1, :), Color=estColor, LineWidth=lw);
    hold on
    plot(mocap_time(mocap_range), mocap_pos(mocap_range, 1), Color=measColor, LineWidth=lw);
    hold off
    ylabel("X (m)")
    
    subplot(312)
    plot(time, outputState(2, :), Color=estColor, LineWidth=lw);
    hold on
    plot(mocap_time(mocap_range), mocap_pos(mocap_range, 2), Color=measColor, LineWidth=lw);
    hold off
    ylabel("Y (m)")
    
    subplot(313)
    plot(time, outputState(3, :), Color=estColor, LineWidth=lw);
    hold on
    plot(mocap_time(mocap_range), mocap_pos(mocap_range, 3), Color=measColor, LineWidth=lw);
    hold off
    ylabel("Z (m)")
    
    xlabel("time (s)");
end

if plot_attitude
    figure()
    subplot(311)
    plot(time, outputState(7, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    if exist('roll_imu','var') == 1
        plot(time, roll_imu(rng), 'k.');
    end
    hold off
    ylabel("\Phi (deg)")
    
    subplot(312)
    plot(time, -outputState(8, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    if exist('pitch_imu','var') == 1
        plot(time, pitch_imu(rng), 'k.');
    end
    hold off
    ylabel("\Theta (deg)")
    
    subplot(313)
    plot(time, -outputState(9, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    if exist('yaw_imu','var') == 1
        plot(time, yaw_imu(rng), 'k.-');
    end
    hold off
    ylabel("\Psi (deg)")
    
    xlabel("time (s)");
end

if plot_velocity
    figure()
    subplot(311)
    plot(time, outputState(4, :), Color=estColor, LineWidth=lw);
    ylabel("Vx (m/s)")
    
    subplot(312)
    plot(time, outputState(5, :), Color=estColor, LineWidth=lw);
    ylabel("Vy (m/s)")
    
    subplot(313)
    plot(time, outputState(6, :), Color=estColor, LineWidth=lw);
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
