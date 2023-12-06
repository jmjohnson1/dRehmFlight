outputState = readtable("outputState.csv");
outputState = table2array(outputState);
time = readtable("imu_time.csv");
time = table2array(time);

dataTable = parseFlightDataAdm("adm11-19-1st-quad.csv");
gyro = [dataTable.GyroX, dataTable.GyroY, dataTable.GyroZ];

C1 = angle2dcm(0, 0, pi);
for i = 1:length(imu_data)
    gyro(i,:) = C1*gyro(i,:)';
end

gyro(:,1) = gyro(:,1) + 0.0213;
gyro(:,2) = gyro(:,2) + 0.0184;

[rollInt, pitchInt, yawInt] = integrateGyro(gyro);

rng = 1601:7600;
time = time(rng);
rollInt = rollInt(rng);
pitchInt = pitchInt(rng);
yawInt = yawInt(rng);
estColor = 'b';
measColor = 'r';
lw = 1;

figure()
s1 = subplot(311);
plot(time, outputState(7, :)*180/pi, Color=estColor, LineWidth=lw);
hold on
plot(time, rollInt, Color=measColor, LineStyle='-')
hold off
ylabel("\Phi (deg)")
grid on;

s2 = subplot(312);
plot(time, outputState(8, :)*180/pi, Color=estColor, LineWidth=lw);
hold on
plot(time, pitchInt, Color=measColor, LineStyle='-')
hold off
ylabel("\Theta (deg)")
grid on;

s3 = subplot(313);
plot(time, outputState(9, :)*180/pi, Color=estColor, LineWidth=lw);
hold on
plot(time, yawInt, Color=measColor, LineStyle='-')
hold off
ylabel("\Psi (deg)")
grid on;

xlabel("time (s)");
linkaxes([s1, s2, s3], 'x')