clear; close all;

rmse_varyAccSigma = table2array(readtable('rmse_varyAccSigma.csv'));
rmse_varyAccMarkov = table2array(readtable('rmse_varyAccMarkov.csv'));
rmse_varyGyroSigma = table2array(readtable('rmse_varyGyroSigma.csv'));
rmse_varyGyroMarkov = table2array(readtable('rmse_varyGyroMarkov.csv'));
rmse_varyAll = table2array(readtable('rmse_varyAll.csv'));
rmse_varyXMarkov = table2array(readtable("rmse_varyXMarkov.csv"));
rmse_varyYMarkov = table2array(readtable("rmse_varyYMarkov.csv"));
rmse_varyZMarkov = table2array(readtable("rmse_varyZMarkov.csv"));
rmse_varyZMarkov_more = table2array(readtable("rmse_varyZMarkov_more.csv"));
rmse_varyZMarkov_morer = table2array(readtable("rmse_varyZMarkov_morer.csv"));
rmse_varyZSigma = table2array(readtable("rmse_varyZSigma.csv"));
rmse_varyZSigma_more = table2array(readtable("rmse_varyZSigma_more.csv"));
rmse_scalePosNE = table2array(readtable("rmse_scalePosNE.csv"));
rmse_scalePosD = table2array(readtable("rmse_scalePosD.csv"));
rmse_scalePosBoth = table2array(readtable("rmse_scalePosBoth.csv"));


for i = 1:3
    h(i) = subplot(3, 1, i);
end

% addPlot(h, rmse_varyAccSigma, "Vary acc sigma");
% addPlot(h, rmse_varyAccMarkov, "Vary acc Markov");
% addPlot(h, rmse_varyGyroSigma, "Vary gyro sigma");
% addPlot(h, rmse_varyGyroMarkov, "Vary gyro Markov");
% addPlot(h, rmse_varyAll, "Vary all");
% addPlot(h, rmse_varyXMarkov, "Vary x markov");
% addPlot(h, rmse_varyYMarkov, "Vary y markov");
% addPlot(h, rmse_varyZMarkov, "Vary z markov");
% addPlot(h, rmse_varyZMarkov_more, "Vary z markov");
% addPlot(h, rmse_varyZMarkov_morer, "Vary z markov");
% addPlot(h, rmse_varyZSigma, "Vary z sigma");
% addPlot(h, rmse_varyZSigma_more, "Vary z sigma");

addPlot(h, rmse_scalePosNE, "Vary pos NE")
addPlot(h, rmse_scalePosD, "Vary pos D")
addPlot(h, rmse_scalePosBoth, "Vary pos Both")

legend(h(1));
xlabel(h, "Scale factor");
ylabel(h(1), "rmseX");
ylabel(h(2), "rmseY");
ylabel(h(3), "rmseZ");

%linkaxes(h, 'y');
set(h, 'ylim', [0 0.3])

function addPlot(h, x, name)
    for i = 2:4
        hold(h(i-1), "on");
        plot(h(i-1), x(:, 1), x(:, i), DisplayName=name);
        hold(h(i-1), "off");
    end
end