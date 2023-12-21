function h = plot_subplots(x1, y1, x2, y2)
%%%
% SYNOPSIS
%   h = plot_subplots(x1, y1, x2, y2)
% DESCRIPTION
%   Takes x and y data in matrices and plots a different subplot for each
%   row. If a second set of x and y are provided, then they must have the
%   same dimensions as the first and will be put on the same subplots.
% INPUTS
% x1       (n, 1)               x data
% y1       (n, m)               y data
% x2       (l, 1)               second set of x data
% y2       (l, m)               second set of y data
% OUTPUTS
% h         (1, m)              axis handles
%%%
    [n, m] = size(y1);
    if nargin < 3
        comparison = 0;
    else
        comparison = 1;
        [l, m2] = size(y2);

        if (m2 ~= m)
        error("y1 and y2 must have the same number of columns");
        end
    end

    lw = 1;
    
    figure();
    for (k = 1:m)
        h(k) = subplot(m, 1, k);
        plot(x1, y1(:, k), LineWidth=lw);
        grid on
        if (comparison)
            hold on
            plot(x2, y2(:, k), LineWidth=lw);
            hold off
        end
    end

end