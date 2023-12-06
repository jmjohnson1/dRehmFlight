%function plotFlightData(filename)
%% Imports
clear
close all
filename = "adm11-19-1st-quad.csv";
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% IMPORT DATA FROM FILE %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %[roll_imu, pitch_imu, yaw_imu, alpha1, beta1, roll_des, pitch_des, yaw_des, throttle_des, roll_pid, pitch_pid, yaw_pid, radio_ch1, radio_ch2, radio_ch3, radio_ch4, radio_ch5, radio_ch6, radio_ch7, radio_ch8, radio_ch9, radio_ch10, radio_ch11, radio_ch12, radio_ch13, GyroX, GyroY, GyroZ, AccX, AccY, AccZ, s1_command, s2_command, s3_command, s4_command, kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw, failsafeTriggered, kp_alphaRoll, ki_alphaRoll, kd_alphaRoll, kp_betaPitch, ki_betaPitch, kd_betaPitch, ripIMU_roll, ripIMU_pitch, ripRoll_des, ripPitch_des, error_alphaRoll, integral_alphaRoll, derivative_alphaRoll, error_betaPitch, integral_betaPitch, derivative_betaPitch] = importfile(filename);
    tbl = readtable(filename);
    roll_imu = table2array(tbl(:, 1)); 
    pitch_imu = table2array(tbl(:, 2)); 
    yaw_imu = table2array(tbl(:, 3));
    roll_des = table2array(tbl(:, 4));
    pitch_des = table2array(tbl(:, 5));
    yaw_des = table2array(tbl(:, 6));
    throttle_des = table2array(tbl(:, 7));
    roll_pid = table2array(tbl(:, 8));
    pitch_pid = table2array(tbl(:, 9));
    yaw_pid = table2array(tbl(:, 10));
    radio_ch1 = table2array(tbl(:, 11));
    radio_ch2 = table2array(tbl(:, 12));
    radio_ch3 = table2array(tbl(:, 13));
    radio_ch4 = table2array(tbl(:, 14));
    radio_ch5 = table2array(tbl(:, 15));
    radio_ch6 = table2array(tbl(:, 16));
    radio_ch7 = table2array(tbl(:, 17));
    radio_ch8 = table2array(tbl(:, 18));
    radio_ch9 = table2array(tbl(:, 19));
    radio_ch10 = table2array(tbl(:, 20));
    radio_ch11 = table2array(tbl(:, 21));
    radio_ch12 = table2array(tbl(:, 22));
    radio_ch13 = table2array(tbl(:, 23));
    GyroX = table2array(tbl(:, 24));
    GyroY = table2array(tbl(:, 25));
    GyroZ = table2array(tbl(:, 26));
    AccX = table2array(tbl(:, 27));
    AccY = table2array(tbl(:, 28));
    AccZ = table2array(tbl(:, 29));
    s1_command = table2array(tbl(:, 30));
    s2_command = table2array(tbl(:, 31));
    s3_command = table2array(tbl(:, 32));
    s4_command = table2array(tbl(:, 33));
    kp_roll = table2array(tbl(:, 34));
    ki_roll = table2array(tbl(:, 35));
    kd_roll = table2array(tbl(:, 36));
    kp_pitch = table2array(tbl(:, 37));
    ki_pitch = table2array(tbl(:, 38));
    kd_pitch = table2array(tbl(:, 39));
    kp_yaw = table2array(tbl(:, 40));
    ki_yaw = table2array(tbl(:, 41));
    kd_yaw = table2array(tbl(:, 42));
    failsafeTriggered = table2array(tbl(:, 43));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%% END IMPORT %%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main
    %t0 = 59;
    t0 = 0;

    lw = 1;
    rng = 1:length(radio_ch1);
    %rng = 5900:36000;
    time_imu = rng/100 - t0;
    %

    imu_data = [AccX, AccY, AccZ, GyroX, GyroY, GyroZ];
    imu_data = imu_data(rng, :);

    C1 = angle2dcm(0, 0, pi);
    for i = 1:length(imu_data)
        imu_data(i, 1:3) = C1*imu_data(i, 1:3)';
        imu_data(i, 4:6) = C1*imu_data(i, 4:6)';
    end

    save("processedIMU.mat", "imu_data", "time_imu");
    

%     %%% Plot Desired and Measured States %%%
    % figure(1);
    % hold on
    % plot(time(rng), roll_des(rng), DisplayName="Desired roll", LineWidth=lw);
    % plot(time(rng), roll_imu(rng), DisplayName="Measured roll", LineWidth=lw);
    % % plot(pitch_des(rng), DisplayName="Desired pitch", LineWidth=lw);
    % % plot(pitch_imu(rng), DisplayName="Measured pitch", LineWidth=lw);
    % % plot(time(rng), yaw_des(rng), DisplayName="Desired yaw", LineWidth=lw);
    % % plot(time(rng), yaw_imu(rng), DisplayName="Measured yaw", LineWidth=lw);
    % hold off
    % legend();
    % grid on
    % title("Pitch/Yaw/Roll")
% 
    % Plot motor commands
%     figure(2);
%     hold on
%     plot(time, s1_command(rng), DisplayName="s1", LineWidth=lw);
%     plot(time, s2_command(rng), DisplayName="s2", LineWidth=lw);
%     plot(time, s3_command(rng), DisplayName="s3", LineWidth=lw);
%     plot(time, s4_command(rng), DisplayName="s4", LineWidth=lw);
%     hold off
%     legend();
%     grid on
%     title("Motor Commands")
% % 
    % % Plot radio channel data
    % figure();
    % hold on
    % plot(radio_ch1(rng), DisplayName="thro", LineWidth=lw)
    % plot(radio_ch2(rng), DisplayName="roll", LineWidth=lw)
    % plot(radio_ch3(rng), DisplayName="pitch", LineWidth=lw)
    % %plot(radio_ch4(rng), DisplayName="yaw", LineWidth=lw)
    % hold off
    % legend();
    % grid on
    % title("Recieved Radio Commands")
% 
%     % Calculate derivative of radio commands (dt = 1/(100 Hz))
%     ch1_diff = diff(radio_ch1(rng))*100;
%     ch2_diff = diff(radio_ch2(rng))*100;
%     ch3_diff = diff(radio_ch3(rng))*100;
%     ch4_diff = diff(radio_ch4(rng))*100;
% 
%     % % Plot radio channel derivatives
%     figure(4);
%     hold on
%     plot(ch1_diff, DisplayName="thro", LineWidth=lw)
%     plot(ch2_diff, DisplayName="roll", LineWidth=lw)
%     plot(ch3_diff, DisplayName="pitch", LineWidth=lw)
%     plot(ch4_diff, DisplayName="yaw", LineWidth=lw)
%     hold off
%     legend();
%     grid on
%     title("Derivative of Recieved Radio Commands (dPWM/dt)")
% 
%     % Plot gains
%     figure(5);
%     hold on
%     plot(kp_roll(rng), DisplayName="K_p_{roll}")
%     plot(ki_roll(rng), DisplayName="K_i_{roll}")
%     plot(kd_roll(rng), DisplayName="K_d_{roll}")
%     plot(kp_pitch(rng), DisplayName="K_p_{pitch}")
%     plot(ki_pitch(rng), DisplayName="K_i_{pitch}")
%     plot(kd_pitch(rng), DisplayName="K_d_{pitch}")
%     hold off
%     legend();
%     grid on
%     title("Gains")
% 
    % Plot PID
    % figure(6);
    % hold on
    % plot(roll_pid(rng), DisplayName="Roll PID");
    % plot(pitch_pid(rng), DisplayName="Pitch PID");
    % hold off
    % legend();
    % grid on
    % title("Normalized PID outputs")
% 
%     % % Plot Gyro
%     figure(7);
%     hold on
%     plot(GyroX(rng), DisplayName="GyroX");
%     plot(GyroY(rng), DisplayName="GyroY");
%     plot(GyroZ(rng), DisplayName="GyroZ");
%     hold off;
%     legend();
%     grid on;
%     title("Gyro");
% 
% %end

% figure()
% ax_roll = subplot(2, 1, 1);
% plot(time_imu, roll_imu(rng), 'r-', DisplayName="Measured Roll")
% hold on
% plot(time_imu, roll_des(rng), 'b-', DisplayName="Desired Roll")
% hold off
% grid on
% ylim([-7, 7])
% legend();
% 
% ax_gains = subplot(2, 1, 2);
% plot(time_imu, kp_roll(rng), 'r-', DisplayName = "Kp")
% hold on
% plot(time_imu, ki_roll(rng), '-', Color=[0 0.5 0], DisplayName = "Ki")
% plot(time_imu, kd_roll(rng), 'b-', DisplayName = "Kd")
% hold off
% grid on
% legend()
% linkaxes([ax_roll, ax_gains], 'x')

% figure()
% ax_roll = subplot(311);
% plot(time_imu, roll_imu(rng), 'r-')
% grid on;
% ax_pitch = subplot(312);
% plot(time_imu, pitch_imu(rng), 'r-')
% grid on;
% ax_yaw = subplot(313);
% plot(time_imu, yaw_imu(rng), 'r-')
% grid on;



% figure()
% ax_ripRoll = subplot(2,1,1);
% plot(time, error_alphaRoll, 'k-', DisplayName="Measured RIP Roll Error");
% hold on
% plot(rng./100, kp_alphaRoll.*error_alphaRoll, 'r-', DisplayName="pTerm");
% plot(rng./100, ki_alphaRoll.*integral_alphaRoll, 'b-', DisplayName="iTerm");
% plot(rng./100, kd_alphaRoll.*derivative_alphaRoll, LineStyle='-', Color=[0 0.5 0], DisplayName="dTerm");
% plot(rng./100, kp_alphaRoll.*error_alphaRoll + ki_alphaRoll.*integral_alphaRoll + kd_alphaRoll.*derivative_alphaRoll, 'r--', DisplayName="Sum");
% hold off;
% grid on;
% legend();
% 
% ax_ripPitch = subplot(2,1,2);
% plot(rng./100, error_betaPitch, 'k-', DisplayName="Measured RIP Pitch Error");
% hold on
% plot(rng./100, kp_betaPitch.*error_betaPitch, 'r-', DisplayName="pTerm");
% plot(rng./100, ki_betaPitch.*integral_betaPitch, 'b-', DisplayName="iTerm");
% plot(rng./100, kd_betaPitch.*derivative_betaPitch, LineStyle='-', Color=[0 0.5 0], DisplayName="dTerm");
% plot(rng./100, kp_betaPitch.*error_betaPitch+ki_betaPitch.*integral_betaPitch+kd_betaPitch.*derivative_betaPitch, 'r--', DisplayName="Sum");
% hold off;
% grid on;
% legend();
% linkaxes([ax_ripRoll, ax_ripPitch], 'xy');
% 
% figure()
% ax1 = subplot(3, 1, 1);
% plot(rng./100, kp_alphaRoll, 'r-', DisplayName="Kp");
% ylabel("Kp");
% grid on;
% 
% ax2 = subplot(3,1,2);
% plot(rng./100, ki_alphaRoll, Color=[0 0.5 0], DisplayName='Ki');
% ylabel("Ki");
% grid on;
% 
% ax3 = subplot(3,1,3);
% plot(rng./100, kd_alphaRoll, 'b-');
% ylabel("Kd");
% grid on;
% linkaxes([ax1, ax2, ax3], 'x');

figure()
subplot(231)
plot(time_imu, imu_data(:, 1));
ylabel("AccX");
subplot(232)
plot(time_imu, imu_data(:, 2));
ylabel("AccY");
subplot(233)
plot(time_imu, imu_data(:, 3));
ylabel("AccZ");
subplot(234)
plot(time_imu, imu_data(:, 4));
ylabel("GyroX");
subplot(235)
plot(time_imu, imu_data(:, 5));
ylabel("GyroY");
subplot(236)
plot(time_imu, imu_data(:, 6));
ylabel("GyroZ");



