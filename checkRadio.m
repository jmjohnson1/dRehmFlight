function y_new = checkRadio(y)
    dt = 1/100;
    y_new = zeros(size(y));
    y_new(1) = y(1);
    for i = 2:length(y)
        dy = y(i) - y_new(i - 1);
        if abs(dy/dt) > 6000
            y_new(i) = y_new(i - 1);
        else
            y_new(i) = y(i);
        end
    end
end