function plotFlightData(fileName)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% IMPORT DATA FROM FILE %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Set up the Import Options and import the data
    opts = delimitedTextImportOptions("NumVariables", 39);
    
    % Specify range and delimiter
    opts.DataLines = [2, Inf];
    opts.Delimiter = ",";
    
    % Specify column names and types
    opts.VariableNames = ["roll_imu", "pitch_imu", "yaw_imu", "alpha1", "beta1", "roll_des", "pitch_des", "yaw_des", "throttle_des", "roll_pid", "pitch_pid", "yaw_pid", "radio_ch1", "radio_ch2", "radio_ch3", "radio_ch4", "radio_ch5", "radio_ch6", "radio_ch7", "GyroX", "GyroY", "GyroZ", "AccX", "AccY", "AccZ", "s1_command", "s2_command", "s3_command", "s4_command", "kp_roll", "ki_roll", "kd_roll", "kp_pitch", "ki_pitch", "kd_pitch", "kp_yaw", "ki_yaw", "kd_yaw", "failsafeTriggered"];
    opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
    
    % Specify file level properties
    opts.ExtraColumnsRule = "ignore";
    opts.EmptyLineRule = "read";
    
    % Import the data
    tbl = readtable(fileName, opts);
    
    % Convert to output type
    roll_imu = tbl.roll_imu;
    pitch_imu = tbl.pitch_imu;
    yaw_imu = tbl.yaw_imu;
    alpha1 = tbl.alpha1;
    beta1 = tbl.beta1;
    roll_des = tbl.roll_des;
    pitch_des = tbl.pitch_des;
    yaw_des = tbl.yaw_des;
    throttle_des = tbl.throttle_des;
    roll_pid = tbl.roll_pid;
    pitch_pid = tbl.pitch_pid;
    yaw_pid = tbl.yaw_pid;
    radio_ch1 = tbl.radio_ch1;
    radio_ch2 = tbl.radio_ch2;
    radio_ch3 = tbl.radio_ch3;
    radio_ch4 = tbl.radio_ch4;
    radio_ch5 = tbl.radio_ch5;
    radio_ch6 = tbl.radio_ch6;
    radio_ch7 = tbl.radio_ch7;
    GyroX = tbl.GyroX;
    GyroY = tbl.GyroY;
    GyroZ = tbl.GyroZ;
    AccX = tbl.AccX;
    AccY = tbl.AccY;
    AccZ = tbl.AccZ;
    s1_command = tbl.s1_command;
    s2_command = tbl.s2_command;
    s3_command = tbl.s3_command;
    s4_command = tbl.s4_command;
    kp_roll = tbl.kp_roll;
    ki_roll = tbl.ki_roll;
    kd_roll = tbl.kd_roll;
    kp_pitch = tbl.kp_pitch;
    ki_pitch = tbl.ki_pitch;
    kd_pitch = tbl.kd_pitch;
    kp_yaw = tbl.kp_yaw;
    ki_yaw = tbl.ki_yaw;
    kd_yaw = tbl.kd_yaw;
    failsafeTriggered = tbl.failsafeTriggered;

    % Clear temporary variables
    clear opts tbl
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%% END IMPORT %%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    lw = 1;
    rng = 1:length(radio_ch1);
    close all

    %%% Plot Desired and Measured States %%%
    figure(1);
    hold on
    plot(roll_des(rng), DisplayName="Desired roll", LineWidth=lw);
    plot(roll_imu(rng), DisplayName="Measured roll", LineWidth=lw);
    % plot(pitch_des(rng), DisplayName="Desired pitch", LineWidth=lw);
    % plot(pitch_imu(rng), DisplayName="Measured pitch", LineWidth=lw);
    % plot(yaw_des(rng), DisplayName="Desired yaw", LineWidth=lw);
    % plot(yaw_imu(rng), DisplayName="Measured yaw", LineWidth=lw);
    hold off
    legend();
    grid on
    title("Pitch/Yaw/Roll")

    % Plot motor commands
    figure(2);
    hold on
    plot(s1_command(rng), DisplayName="s1", LineWidth=lw);
    plot(s2_command(rng), DisplayName="s2", LineWidth=lw);
    plot(s3_command(rng), DisplayName="s3", LineWidth=lw);
    plot(s4_command(rng), DisplayName="s4", LineWidth=lw);
    hold off
    legend();
    grid on
    title("Motor Commands")

    % Plot radio channel data
    figure(3);
    hold on
    plot(radio_ch1(rng), DisplayName="thro", LineWidth=lw)
    plot(radio_ch2(rng), DisplayName="roll", LineWidth=lw)
    plot(radio_ch3(rng), DisplayName="pitch", LineWidth=lw)
    plot(radio_ch4(rng), DisplayName="yaw", LineWidth=lw)
    hold off
    legend();
    grid on
    title("Recieved Radio Commands")

    % Calculate derivative of radio commands (dt = 1/(100 Hz))
    ch1_diff = diff(radio_ch1(rng))*100;
    ch2_diff = diff(radio_ch2(rng))*100;
    ch3_diff = diff(radio_ch3(rng))*100;
    ch4_diff = diff(radio_ch4(rng))*100;

    % Plot radio channel derivatives
    figure(4);
    hold on
    plot(ch1_diff, DisplayName="thro", LineWidth=lw)
    plot(ch2_diff, DisplayName="roll", LineWidth=lw)
    plot(ch3_diff, DisplayName="pitch", LineWidth=lw)
    plot(ch4_diff, DisplayName="yaw", LineWidth=lw)
    hold off
    legend();
    grid on
    title("Derivative of Recieved Radio Commands (dPWM/dt)")

end