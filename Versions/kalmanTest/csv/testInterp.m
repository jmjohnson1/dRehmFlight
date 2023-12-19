imuTime = readtable("imu_time.csv");
mocapPos = readtable("mocapPos_debug.csv");
mocapTime = readtable("mocap_time.csv");
x_i = readtable("xInterp_test.csv", 'Delimiter', ',');

imuTime = table2array(imuTime);
mocapPos = table2array(mocapPos);
mocapTime = table2array(mocapTime);
x_i = table2array(x_i);

plot(mocapTime, mocapPos(:, 1), DisplayName="Original"); hold on;
plot(imuTime, x_i, DisplayName="interpolated");
hold off;
legend()
grid on

