%% Definitions
I = 10;
K_p = 16;
K_d = 10;


%% tf Method
% Define transfer function numerator/denominator coefficients
num = [K_d K_p];
den = [I K_d K_p];
sys = tf(num, den);

% create time vector
t = linspace(0, 10, 100);

% apply step input to system
[theta_tf, t] = step(sys, t);

% plot
plot(t, theta_tf)
hold on

%% ode45 Method

% here we have to define theta_r, theta_dot_r explicitly
theta_r = 1;
theta_dot_r = 0;

y0 = [0; 1];

[t, y] = ode45(@(t, y) dydt(t, y, I, K_p, K_d, theta_r, theta_dot_r),...
    [0 10], y0);

% unpack
theta_ode = y(:, 1);
theta_dot_ode = y(:, 2);

plot(t, theta_ode)

function dydt = dydt(t, y, I, K_p, K_d, theta_r, theta_dot_r)
    % unpack y vector
    theta = y(1);
    theta_dot = y(2);
    
    % preallocate derivative vector
    dydt = zeros(2, 1);
    
    % derivative of theta state
    dydt(1) = theta_dot;
    
    % derivative of theta_dot state
    dydt(2) = 1 / I * (- K_p * (theta - theta_r) ...
                       - K_d * (theta_dot - theta_dot_r));
end