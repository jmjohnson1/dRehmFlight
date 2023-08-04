%function plotFlightData(filename)
filename = "flight_data16.csv";
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% IMPORT DATA FROM FILE %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [roll_imu, pitch_imu, yaw_imu, alpha1, beta1, roll_des, pitch_des, yaw_des, throttle_des, roll_pid, pitch_pid, yaw_pid, radio_ch1, radio_ch2, radio_ch3, radio_ch4, radio_ch5, radio_ch6, radio_ch7, radio_ch8, radio_ch9, radio_ch10, radio_ch11, radio_ch12, radio_ch13, GyroX, GyroY, GyroZ, AccX, AccY, AccZ, s1_command, s2_command, s3_command, s4_command, kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw, failsafeTriggered, kp_alphaRoll, ki_alphaRoll, kd_alphaRoll, kp_betaPitch, ki_betaPitch, kd_betaPitch, ripIMU_roll, ripIMU_pitch] = importfile(filename);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%% END IMPORT %%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    lw = 1;
     rng = 1:length(radio_ch1);
    %rng = 5.10E04:5.15E04;
    time = rng/100;
    %
    
    close all

%     %%% Plot Desired and Measured States %%%
    figure(1);
    hold on
    plot(roll_des(rng), DisplayName="Desired roll", LineWidth=lw);
    plot(roll_imu(rng), DisplayName="Measured roll", LineWidth=lw);
    % plot(pitch_des(rng), DisplayName="Desired pitch", LineWidth=lw);
    % plot(pitch_imu(rng), DisplayName="Measured pitch", LineWidth=lw);
    % plot(time(rng), yaw_des(rng), DisplayName="Desired yaw", LineWidth=lw);
    % plot(time(rng), yaw_imu(rng), DisplayName="Measured yaw", LineWidth=lw);

    plot(beta1(rng)+pitch_imu(rng), DisplayName="Beta+pitch", LineWidth=lw);
    hold off
    legend();
    grid on
    title("Pitch/Yaw/Roll")
% 
%     % Plot motor commands
%     figure(2);
%     hold on
%     plot(s1_command(rng), DisplayName="s1", LineWidth=lw);
%     plot(s2_command(rng), DisplayName="s2", LineWidth=lw);
%     plot(s3_command(rng), DisplayName="s3", LineWidth=lw);
%     plot(s4_command(rng), DisplayName="s4", LineWidth=lw);
%     hold off
%     legend();
%     grid on
%     title("Motor Commands")
% 
%     % % Plot radio channel data
%     figure(3);
%     hold on
%     plot(radio_ch11(rng), DisplayName="thro", LineWidth=lw)
%     plot(radio_ch12(rng), DisplayName="roll", LineWidth=lw)
%     plot(radio_ch13(rng), DisplayName="pitch", LineWidth=lw)
%     %plot(radio_ch4(rng), DisplayName="yaw", LineWidth=lw)
%     hold off
%     legend();
%     grid on
%     title("Recieved Radio Commands")
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
    % % Plot PID
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

figure()
subplot(2, 1, 1)
plot(rng./100, roll_imu(rng), 'r-', DisplayName="Measured Roll")
hold on
plot(rng./100, roll_des(rng), 'b-', DisplayName="Desired Roll")
hold off
grid on
ylim([-7, 7])
legend();

subplot(2, 1, 2)
plot(rng./100, kp_roll(rng), 'r-', DisplayName = "Kp")
hold on
plot(rng./100, ki_roll(rng), '-', Color=[0 0.5 0], DisplayName = "Ki")
plot(rng./100, kd_roll(rng), 'b-', DisplayName = "Kd")
hold off
grid on
legend()

% figure()
% plot(time, alpha1, DisplayName="alpha");
% hold on
% plot(time, alpha1+roll_imu, DisplayName="alphaRoll");
% plot(time, beta1, DisplayName="beta");
% plot(time, beta1+pitch_imu, DisplayName="betaPitch");
% plot(time, ripIMU_roll, DisplayName="ripIMU_roll");
% plot(time, ripIMU_pitch, DisplayName="ripIMU_pitch");
% hold off
% legend()
% grid on;

% motor.xOffset = 3;
% motor.yOffset = 3;
% motor.radius = 0.25;
% 
% 
% idx = 5000;
% th = 0:360;
% % Draw iris circle
% plot(cosd(th), sind(th), 'r-', DisplayName="Iris Limits");
% grid on;
% axis equal;
% 
% % Draw motors
% hold on;
% m2.plot = plot((motor.radius*cosd(th) + motor.xOffset), ...
%     (motor.radius*sind(th) + motor.yOffset), 'k-');
% 
% m1.plot = plot(-(motor.radius*cosd(th) + motor.xOffset), ...
%     (motor.radius*sind(th) + motor.yOffset), 'k-');
% 
% m3.plot = plot((motor.radius*cosd(th) + motor.xOffset), ...
%     -(motor.radius*sind(th) + motor.yOffset), 'k-');
% 
% m4.plot = plot(-(motor.radius*cosd(th) + motor.xOffset), ...
%     -(motor.radius*sind(th) + motor.yOffset), 'k-');
% hold off;
% 
% % Label values in motors
% m1.text = text(motor.xOffset, motor.yOffset, string(s1_command(idx)));
% m2.text = text(motor.xOffset, -motor.yOffset, string(s2_command(idx)));
% m3.text = text(-motor.xOffset, -motor.yOffset, string(s3_command(idx)));
% m4.text = text(-motor.xOffset, motor.yOffset, string(s4_command(idx)));
% 
% % Plot iso
% r1 = sind(5)/sind(20);
% r2 = sind(10)/sind(20);
% r3 = sind(15)/sind(20);
% hold on;
% plot(r1*cosd(th), r1*sind(th), 'k--');
% plot(r2*cosd(th), r2*sind(th), 'k--');
% plot(r3*cosd(th), r3*sind(th), 'k--');
% hold off;
% 
% % Place the pendulum
% pendY = -sind(alpha1(idx))/sind(20);
% pendX = sind(beta1(idx))/sind(20);
% 
% hold on;
% plot(pendX, pendY, 'b.', MarkerSize=20);
% hold off;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%