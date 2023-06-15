clear
clc

sp = serialport("/dev/ttyACM1", 500000);

data1 = zeros(3000, 1);
data2 = zeros(3000, 1);
data3 = zeros(3000, 1);
data4 = zeros(3000, 1);
data5 = zeros(3000, 1);
data6 = zeros(3000, 1);


%Set up Plot
fig = figure('WindowStyle', 'docked');
hold on;
plotGraph1 = plot(data1,'-r', DisplayName="\alpha");
plotGraph2 = plot(data2,'-b', DisplayName="roll");
plotGraph3 = plot(data3, DisplayName='roll + \alpha');
plotGraph4 = plot(data4, DisplayName='\beta');
plotGraph5 = plot(data5, DisplayName='pitch');
plotGraph6 = plot(data6, DisplayName='pitch + \beta');
hold off
title("Serial Data",'FontSize',15);
ylabel("Angle (deg)",'FontSize',15);
legend()
axis([0 3000 -15 15]);
grid("on");
grid minor

plotGraph1.YDataSource = 'data1';
plotGraph2.YDataSource = 'data2';
plotGraph3.YDataSource = 'data3';
plotGraph4.YDataSource = 'data4';
plotGraph5.YDataSource = 'data5';
plotGraph6.YDataSource = 'data6';

flush(sp);

while 1
    dataString = readline(sp);
    dataLine = str2double(strsplit(dataString));
    data1 = circshift(data1, -1);
    data1(end) = dataLine(1);
    data2 = circshift(data2, -1);
    data2(end) = dataLine(2);
    data3 = circshift(data3, -1);
    data3(end) = dataLine(3);
    data4 = circshift(data4, -1);
    data4(end) = dataLine(4);
    data5 = circshift(data5, -1);
    data5(end) = dataLine(5);
    data6 = circshift(data6, -1);
    data6(end) = dataLine(6);
    refreshdata(plotGraph1);
    refreshdata(plotGraph2);
    refreshdata(plotGraph3);
    refreshdata(plotGraph4);
    refreshdata(plotGraph5);
    refreshdata(plotGraph6);
end

clear sp;
