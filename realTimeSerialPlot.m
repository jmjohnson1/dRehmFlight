clear
clc

sp = serialport("/dev/ttyACM1", 500000);

data1 = zeros(3000, 1);
data2 = zeros(3000, 1);


%Set up Plot
fig = figure('WindowStyle', 'docked');
hold on;
plotGraph1 = plot(data1,'-r', DisplayName="\alpha");
plotGraph2 = plot(data2,'-b', DisplayName="roll");
hold off
title("Serial Data",'FontSize',15);
ylabel("Angle (deg)",'FontSize',15);
legend()
axis([0 3000 -30 30]);
grid("on");

plotGraph1.YDataSource = 'data1';
plotGraph2.YDataSource = 'data2';


flush(sp);
while 1
    dataString = readline(sp);
    dataLine = str2double(strsplit(dataString));
    data1 = circshift(data1, -1);
    data1(end) = dataLine(1);
    data2 = circshift(data2, -1);
    data2(end) = dataLine(3);
    refreshdata(plotGraph1);
    refreshdata(plotGraph2);
end

clear sp;