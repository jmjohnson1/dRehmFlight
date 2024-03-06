clear;
close all;

flightData = table2array(readtable("flightData.csv"));
outputState = readtable("outputState.csv");
outputState = table2array(outputState);
innovationCov = table2array(readtable("innovations.csv"));
residual = table2array(readtable("residual.csv"));

load("mocapData.mat");
mocap_pos = mocapPosition_i;


minTime = 0;
maxTime = 1000;
est_idx = find(time >= minTime & time <= maxTime);



estColor = 'b';
measColor = 'r';
lw = 1;

plot_position = true;
plot_attitude = true;
plot_velocity = false;
plot_biases = true;
plot_posError = true;


time = time(est_idx);
outputState = outputState(:, est_idx);
mocap_pos = mocap_pos(est_idx, :);
flightData = flightData(est_idx, :);
innovationCov = innovationCov(:, est_idx);
residual = residual(:, est_idx);

% measurement updates
measUpdate = find(diff(flightData(:,11)) > 0) + 1;
N = length(measUpdate);

% Calculate normalized innovation error squared
nis = zeros(1, N);
for LV1 = 1:N
  k = measUpdate(LV1);
  Sk = diag(innovationCov(:, k));
  yk = residual(:, k);
  nis(LV1) = yk'*inv(Sk)*yk;
end



% Number of samples within 2sigma bound
numBounded = [0; 0; 0];
for LV1 = 1:length(measUpdate)
  k = measUpdate(LV1);
  yay = abs(residual(:, k)) < 2*sqrt(innovationCov(:, k));
  numBounded = numBounded + yay;
end
fracBounded = numBounded/N


errorX = mocap_pos(:, 1) - outputState(1, :)';
errorY = mocap_pos(:, 2) - outputState(2, :)';
errorZ = mocap_pos(:, 3) - outputState(3, :)';

rmseX = rms(errorX, "omitnan");
rmseY = rms(errorY, "omitnan");
rmseZ = rms(errorZ, "omitnan");


if plot_position
    figure()
    s1 = subplot(311);
    plot(time, outputState(1, :), Color=estColor, LineWidth=lw);
    hold on
    plot(time, mocap_pos(:, 1), Color=measColor, LineWidth=lw);
    hold off
    ylabel("X (m)")
    grid on;
    
    
    s2 = subplot(312);
    plot(time, outputState(2, :), Color=estColor, LineWidth=lw);
    hold on
    plot(time, mocap_pos(:, 2), Color=measColor, LineWidth=lw);
    hold off
    ylabel("Y (m)")
    grid on;
    
    s3 = subplot(313);
    plot(time, outputState(3, :), Color=estColor, LineWidth=lw);
    hold on
    plot(time, mocap_pos(:, 3), Color=measColor, LineWidth=lw);
    hold off
    ylabel("Z (m)")
    grid on;
    legend("Estimated position", "Measured position", Location="southeast")
    
    xlabel("time (s)");
    linkaxes([s1, s2, s3], 'x')
end

if plot_attitude
    figure()
    s1 = subplot(311);
    plot(time, outputState(7, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    plot(time, flightData(:, 15), Color=measColor, LineStyle='-')
    hold off
    ylabel("\Phi (deg)")
    grid on;
    
    s2 = subplot(312);
    plot(time, outputState(8, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    plot(time, flightData(:, 16), Color=measColor, LineStyle='-')
    hold off
    ylabel("\Theta (deg)")
    grid on;
    
    s3 = subplot(313);
    plot(time, outputState(9, :)*180/pi, Color=estColor, LineWidth=lw);
    hold on
    plot(time, -flightData(:, 17)*180/pi, Color=measColor, LineStyle='-')
    hold off
    ylabel("\Psi (deg)")
    grid on;
    
    xlabel("time (s)");
    linkaxes([s1, s2, s3], 'x')

end


if plot_velocity
    figure()
    subplot(311)
    plot(time, outputState(4, :), Color=estColor, LineWidth=lw);
    ylabel("Vx (m/s)")
    grid on;
    
    subplot(312)
    plot(time, outputState(5, :), Color=estColor, LineWidth=lw);
    ylabel("Vy (m/s)")
    grid on;
    
    subplot(313)
    plot(time, outputState(6, :), Color=estColor, LineWidth=lw);
    ylabel("Vz (m/s)")
    grid on;
    
    xlabel("time (s)");
end

if plot_biases
    figure()
    subplot(311)
    plot(time, outputState(10, :), Color=estColor, LineWidth=lw);
    title("Acc Biases")
    ylabel("")
    grid on;
    
    subplot(312)
    plot(time, outputState(11, :), Color=estColor, LineWidth=lw);
    ylabel("")
    grid on;
    
    subplot(313)
    plot(time, outputState(12, :), Color=estColor, LineWidth=lw);
    ylabel("")
    grid on;
    
    figure()
    subplot(311)
    plot(time, outputState(13, :), Color=estColor, LineWidth=lw);
    title("Gyro Biases")
    ylabel("")
    grid on;
    
    subplot(312)
    plot(time, outputState(14, :), Color=estColor, LineWidth=lw);
    ylabel("")
    grid on;
    
    subplot(313)
    plot(time, outputState(15, :), Color=estColor, LineWidth=lw);
    ylabel("")
    grid on;
end

if plot_posError
    figure()
    s1 = subplot(311);
    plot(time, errorX, Color=estColor, LineWidth=lw);
    ylabel("X Error (m)")
    grid on;
    
    s2 = subplot(312);
    plot(time, errorY, Color=estColor, LineWidth=lw);
    ylabel("Y Error (m)")
    grid on;
    
    s3 = subplot(313);
    plot(time, errorZ, Color=estColor, LineWidth=lw);
    ylabel("Z Error (m)")
    grid on;
    
    xlabel("time (s)");
    linkaxes([s1, s2, s3], 'x')
end


figure()
  plot(time(measUpdate), nis, DisplayName="NIS");
  hold on
  plot(time(measUpdate), cummean(nis), 'k--');
  hold off
  xlabel("time (s)");
  ylabel("NIS");
 
figure()
  ax(1) = subplot(311);
  plot(time(measUpdate), residual(1, measUpdate), 'b.')
  hold on
  plot(time(measUpdate), 2*sqrt(innovationCov(1, measUpdate)), 'k--');
  plot(time(measUpdate), -2*sqrt(innovationCov(1, measUpdate)), 'k--');
  hold off

  ax(1) = subplot(312);
  plot(time(measUpdate), residual(2, measUpdate), 'b.')
  hold on
  plot(time(measUpdate), 2*sqrt(innovationCov(2, measUpdate)), 'k--');
  plot(time(measUpdate), -2*sqrt(innovationCov(2, measUpdate)), 'k--');
  hold off

  ax(1) = subplot(313);
  plot(time(measUpdate), residual(3, measUpdate), 'b.')
  hold on
  plot(time(measUpdate), 2*sqrt(innovationCov(3, measUpdate)), 'k--');
  plot(time(measUpdate), -2*sqrt(innovationCov(3, measUpdate)), 'k--');
  hold off
