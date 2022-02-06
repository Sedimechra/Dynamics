clc
clear all
%% Definitions
I = 10;
K_p = 15;
K_d = 10;
K_I = 5;
g = 0.5 * K_p;

%% tf Method
% Define transfer function numerator/denominator coefficients
num = [K_p K_I];
den = [1 (0.5+K_p) K_I];
sys = tf(num, den);

% create time vector
t = linspace(0, 5, 100);

% apply step input to system
[theta_tf, t] = step(sys, t);

% plotz
figure
plot(t, theta_tf)
xlabel("Time (seconds)")
ylabel("Amplitude")
hold off

u = K_p .* (1 - theta_tf) + K_I .* (1 - theta_tf).*t;
figure
plot(t,u)
xlabel("Time (seconds)")
ylabel("Force")
stepinfo(sys,t,1,'SettlingTimeThreshold',0.1)
% f = stepinfo(sys,'SettlingTimeThreshold',0.1)
% f.SettlingTime

