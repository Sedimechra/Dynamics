%% Parabolic Path calculations
% input initial velocity, initial position,
% output position vectors, final velocity
function [x , y , z , vf] = Parabola(v0,pos0)
%% Setup of variables and vectors
t = linspace(0,100,100);
g = 9.81;
x = ones(1,length(t)).*4200; % filling with large values
y = ones(1,length(t)).*4200; % to be trimmed later
z = ones(1,length(t)).*4200;
x(1) = pos0(1);
y(1) = pos0(2);
z(1) = pos0(3);
i = 2;
%% Calculating path with kinematics
while i <=length(t)
    x(i) = (v0(1)*t(i))+x(1);
    y(i) = (v0(2)*t(i))+y(1);
    z(i) = ((v0(3)*t(i))-(0.5*g*t(i)*t(i)))+z(1);
    if z(i) <= z(1)
        i = length(t);
    end
    i = i + 1;
end
%% Trimming position vectors
x = x(x ~= 4200);   % output, [meters]
y = y(y ~= 4200);   % output, [meters]
z = z(z ~= 4200);   % output, [meters]
z(end) = z(1);
vf = [v0(1) v0(2) -v0(3)];  % output, [m/s]
end