t = 0:0.01:10;
n = 1;
for i = 0:n
    temp = StepFunction(t - i - 1);
    x = (t - i) - (t - i).*temp;
end

plot(t, x);