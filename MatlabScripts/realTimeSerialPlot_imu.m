clear;
close all;
clc

sp = serialport("/dev/tty.usbmodem130652101", 500000);

Acc = zeros(6000, 3);
Gyro = zeros(6000, 3);


%Set up Plot
fig_acc = figure(1);
fig_gyro = figure(2);

hold on;
plotGraph1 = plot(fig_acc, Acc);
plotGraph2 = plot(fig_gyro, Gyro);
hold off
legend(plotGraph1, 'AccX', 'AccY', 'AccZ');
legend(plotGraph2, 'GyroX', 'GyroY', 'GyroZ');
axis(plotGraph1, [0 6000 -20 20]);
axis(plotGraph2, [0 6000 -260 260]);
grid(plotGraph1, "on");
grid(plotGraph2, "on");

% plotGraph1.YDataSource = 'data1';
% plotGraph2.YDataSource = 'data2';


flush(sp);
while 1
    dataString = readline(sp);
    dataLine = str2double(strsplit(dataString));

    Acc = circshift(AccX, -1);
    Acc(end, :) = dataLine(1:3);
    Gyro = circshift(Gyro, -1);
    Gyro(end, :) = dataLine(4:6);


    % refreshdata(plotGraph1);
    % refreshdata(plotGraph2);
    plotGraph1.YData = Acc;
    plotGraph2.YData = Gyro;
end

clear sp;
