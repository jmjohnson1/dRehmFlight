clear
close all

dataSet = importdata("quad_145554.log");
idx_min = 110;
idx_max = 5828;


Wn = 5/(sampleFrequency/2);
[b, a] = butter(6, Wn, 'low');

outputAngle = dataSet(idx_min:idx_max, 1);
commandAngle = dataSet(idx_min:idx_max, 5);

outputFFT = fft(outputAngle);
commandFFT = fft(commandAngle);


L = idx_max-idx_min + 1;
sampleFrequency = L/60;

time = (idx_min:idx_max)/sampleFrequency;

P2_command = abs(commandFFT/L);
P1_command = P2_command(1:L/2+1);
%P1_command(2:end-1) = 2*P1_command(2:end-1);

P2_output = abs(outputFFT/L);
P1_output = P2_output(1:L/2+1);
%P1_output(2:end-1) = 2*P1_output(2:end-1);

f = sampleFrequency*(0:(L/2))/L;

% figure(1);
% plot(time, outputAngle, 'b', LineWidth=1);
% hold on
% plot(time, commandAngle, 'r', LineWidth=1);
% hold off
% ylabel('Roll Angle (deg)')
% ylim([-11, 11])
% legend("Output", "Command");
half = floor(L/2);

resp = outputFFT./commandFFT;
% Wn = 5/(sampleFrequency/2);
% [b, a] = butter(6, Wn, 'low');
% resp_filtered = filter(b, a, resp(1:half));

figure;
semilogx(f(1:half), 20*log10(abs(resp(1:half))));
xlim([0, 2])
ylim([-5, 5]);

figure;
semilogx(f(1:half), 20*log10(abs(resp_filtered(1:half))));
xlim([0, 2])
ylim([-5, 5])

figure;
semilogx(f(1:half), angle(resp(1:half)));
xlim([0,2])
