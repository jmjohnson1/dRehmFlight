clear
clc

sp = serialport("/dev/tty.usbmodem147193601", 500000);

data1 = zeros(6000, 1);
data2 = zeros(6000, 1);


%Set up Plot
fig = figure();

hold on;
plotGraph1 = plot(data1,'-r', DisplayName="Desired Roll");
plotGraph2 = plot(data2,'-b', DisplayName="Measured Roll");
hold off
title("Serial Data",'FontSize',15);
ylabel("",'FontSize',15);
legend()
axis([0 6000 -20 20]);
grid("on");

% plotGraph1.YDataSource = 'data1';
% plotGraph2.YDataSource = 'data2';


flush(sp);
while 1
    dataString = readline(sp);
    dataLine = str2double(strsplit(dataString));
    data1 = circshift(data1, -1);
    data1(end) = dataLine(1);
    data2 = circshift(data2, -1);
    data2(end) = dataLine(2);
    % refreshdata(plotGraph1);
    % refreshdata(plotGraph2);
    plotGraph1.YData = data1;
    plotGraph2.YData = data2;
end

clear sp;
