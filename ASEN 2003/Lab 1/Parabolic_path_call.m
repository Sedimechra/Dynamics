%% Setup
clc
clear
close all

%% Initialize parameters
v0 = [0 40 10];
pos0 = [4 10 50];
%% Calling function
[x,y,z,vf] = Parabola(v0,pos0);
%% Plotting
plot3(x,y,z,'LineWidth',2);
grid on
xlabel('X');
ylabel('Y');
zlabel('Z');
disp(vf);