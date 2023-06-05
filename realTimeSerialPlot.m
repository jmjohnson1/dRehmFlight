clear
clc

sp = serialport("/dev/ttyACM1", 500000);

data1 = zeros(3000, 1);
data2 = zeros(3000, 1);
flush(sp);

%Set up Plot
plotGraph1 = plot(data1,'-r' );
hold on
plotGraph2 = plot(data2,'-b');
title("Serial Data",'FontSize',15);
ylabel("Angle (deg)",'FontSize',15);
legend()
axis([0 3000 -30 30]);
grid("on");

plotGraph1.YDataSource = 'data1';
plotGraph2.YDataSource = 'data2';

while ishandle(plotGraph1)
    dataString = readline(sp);
    dataLine = str2double(strsplit(dataString));
    data1 = circshift(data1, -1);
    data1(end) = dataLine(1);
    data2 = circshift(data2, -1);
    data2(end) = dataLine(2);
    refreshdata(plotGraph1, 'caller');
    refreshdata(plotGraph2, 'caller');
end

clear sp;

