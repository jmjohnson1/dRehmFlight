function plotFlightData(fileName)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% IMPORT DATA FROM FILE %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Set up the Import Options and import the data
    opts = delimitedTextImportOptions("NumVariables", 29);
    
    % Specify range and delimiter
    opts.DataLines = [2, Inf];
    opts.Delimiter = ",";
    
    % Specify column names and types
    opts.VariableNames = ["roll_imu", "pitch_imu", "yaw_imu", "alpha1", "beta1", "roll_des", "pitch_des", "yaw_des", "throttle_des", "roll_pid", "pitch_pid", "yaw_pid", "radio_ch1", "radio_ch2", "radio_ch3", "radio_ch4", "radio_ch5", "radio_ch6", "radio_ch7", "GyroX", "GyroY", "GyroZ", "AccX", "AccY", "AccZ", "s1_command", "s2_command", "s3_command", "s4_command"];
    opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
    
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
    
    % Clear temporary variables
    clear opts tbl
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%% END IMPORT %%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 
    %%% Plot Desired and Measured States %%%
    desAndMeas_figure = figure();
    des_plot_roll = plot(roll_des, DisplayName="Desired roll");
    hold on
    des_plot_pitch = plot(pitch_des, DisplayName="Desired pitch");
    des_plot_yaw = plot(yaw_des, DisplayName="Desired yaw");
    meas_plot_roll = plot(roll_imu, DisplayName="Measured roll");
    meas_plot_pitch = plot(pitch_imu, DisplayName="Measured pitch");
    meas_plot_yaw = plot(yaw_imu, DisplayName="Measured yaw");
    hold off
    legend();

end