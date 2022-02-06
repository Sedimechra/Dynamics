%% Setup
clc
clear
close
%% Initialize loop parameters
pos0 = [10 10 90];
v0 = [10 0 0];
r = 15;
prop = 1;
%% Call function
[pos,vf,G] = loopPath(pos0,r,prop); % The final velocity is just identical
                                    % the initial for now. This works for
                                    % one full loop
%% Separate positions
x = pos(1,:);
y = pos(2,:);
z = pos(3,:);
%% Plotting position
plot3(x,y,z, 'LineWidth', 2);
hold on
scatter(x(1),y(1),z(1));
grid on
xlabel('X');
ylabel('Y');
zlabel('Z');
xlim([min(x)-5 max(x)+5]);
ylim([min(y)-5 max(y)+5]);
zlim([min(z)-5 max(z)+5]);

%% Plotting g forces
figure (2)
theta = linspace(0,2*pi,1000);
plot(theta,G);
title('G Forces');
xlabel('theta');
ylabel("G's");

