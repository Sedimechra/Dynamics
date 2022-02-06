% ASEN 2003
% Lab 1: Roller Coaster
% Group 3:
% Tristian Workman
% Tyler Schwinck
% Drew Miller
% Max Martinez
%% ---------------------------------------------------------------------- %
% Housekeeping
clc
clear
%% ---------------------------------------------------------------------- %
[x1, y1, z1, vel1, Gx1, Gy1, Gz1] = section1(0,50,125);
[x2, y2, z2, vel2, Gx2, Gy2, Gz2] = section2(x1(end),y1(end),z1(end));
[x3, y3, z3, vel3, Gx3, Gy3, Gz3] = section3(x2(end),y2(end),z2(end),10);
[x4,y4,z4,vel4,Gx4,Gy4,Gz4] = loopPath(x3(end),y3(end),z3(end),8,1);
[x5, y5, z5, vel5, Gx5, Gy5, Gz5] = section3(x4(end),y4(end),z4(end),10);
[x6, y6, z6, vel6, Gx6, Gy6, Gz6] = launch(x5(end),y5(end),z5(end));
parabolicPos = [x6(end),y6(end),z6(end)];
parabolicVel = [vel6(end)*sqrt(2)/2,0,vel6(end)*sqrt(2)/2];
[x7, y7, z7, vel7, Gx7, Gy7, Gz7] = Parabola(parabolicVel,parabolicPos);
[x8, y8, z8, vel8, Gx8, Gy8, Gz8] = landing(x7(end),y7(end),z7(end));
% ----------------------------------------------------------------------- %
coords = valley(30,15,10);
x9 = coords(1,1:end);
y9 = coords(2,1:end);
z9 = coords(3,1:end);
Theta = coords(4,1:end);
Velocity = coords(5,1:end);
Gz9 = coords(6,1:end);
% ----------------------------------------------------------------------- %
figure (1)
plot3(x1,y1,z1,'LineWidth',2) % Initial Drop
hold on
plot3(x2,y2,z2,'LineWidth',2) % Initial Curve
plot3(x3,y3,z3,'LineWidth',2) % Transition to Loop
plot3(x4,y4,z4,'LineWidth',2) % Loop
plot3(x5,y5,z5,'LineWidth',2) % Transition from loop
plot3(x6,y6,z6,'LineWidth',2) % Transition to hill
plot3(x7,y7,z7,'LineWidth',2) % 0-G Parabolic Hill
plot3(x8,y8,z8,'LineWidth',2) % Transition from hill
plot3(x9,y9,z9,'LineWidth',2) % Circular Valley
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")
xlim([0 150])
ylim([0 150])
zlim([0 125])
xlabel('x-Distance (m)')
ylabel('y-Distance (m)')
zlabel('z-Height (m)')
hold off
grid on

figure (2)
plot([Gz1 Gz2 Gz3 Gz4 Gz5 Gz6 Gz7 Gz8 Gz9])
% figure(3)
% plot([vel1 vel2 vel3 vel4 vel5 vel6 vel7])

function [x, y, z, vel, Gx, Gy, Gz] = section1(xI,yI,zI)
%     theta = linspace(pi/2,0,100);
%     radius = 10;
%     g = 9.81;
%     x = radius.*cos(theta) + xI;
%     y = theta.*0 + yI;
%     z = radius.*sin(theta) + zI - radius;
%     vel = sqrt(2 .* g .* (zI - z));
%     Gz = -((vel.^2)./(g * radius)) + sin(theta);
%     Gx = theta .* 0;
%     Gy = theta .* 0;
    t = linspace(0,10,100);
    g = 9.81;
    x = t.*0 + xI;
    y = t.*0 + yI;
    z = -t + zI;
    vel = sqrt(2 .* g .* (125 - z));
    Gz = t .* 0;
    Gx = t .* 0;
    Gy = t .* 0;
end
function [x, y, z, vel, Gx, Gy, Gz] = section2(xI,yI,zI)
    % circular section, G's occur perpendicular to cart, vert -> flat
    theta = linspace(pi,3*pi/2,100);
    radius = 10;
    g = 9.81;
    x = radius.*cos(theta) + radius;
    y = theta.*0 + yI;
    z = radius.*sin(theta) + zI;
    vel = sqrt(2 .* g .* (125 - z));
    Gz = ((vel.^2)./(g * radius)) - sin(theta);
    Gx = theta .* 0;
    Gy = theta .* 0;
end
function [x, y, z, vel, Gx, Gy, Gz] = section3(xI,yI,zI,L)
    % linear section, 1G downwards, propogates in the x direction 
    t = linspace(0,L,100);
    g = 9.81;
    x = t + xI;
    y = t.*0 + yI;
    z = t.*0 + zI;
    vel = sqrt(2 .* g .* (125 - z));
    Gz = t .* 0 + 1;
    Gx = t .* 0;
    Gy = t .* 0;
end
function [x,y,z,V,Gx,Gy,Gz] = loopPath(xI,yI,zI,r,prop)
    %% Setup
    % Note, this currently only calculated velocity correctly
    % if the path is one full loop. I need to add the calculations
    % for the velocity vector for a partial loop.
    h0 = 125; % [m] initial height
    m = 100;  % [kg] particle mass
    g = 9.81; % [m/s^2] gravity
    t = linspace(-pi/2,(prop.*2.*pi)-pi./2,100);
    %% calculating positions
    x = xI + r.*cos(t);
    z = zI + r + r.*sin(t);
    y = yI + 0.*t;         % If you change t to be multiplied by a nonzero number it becomes a helix
    %% velocity and G force calculations
    V = t.*0 + sqrt(2*g*(h0-z));
    N = m*((cos(t)*g)+((V.^2)/r));
    Gz = t.*0 + N/(m*g);
    Gx = t.*0;
    Gy = t.*0;
    %% output
end
function [x, y, z, vel, Gx, Gy, Gz] = launch(xI,yI,zI)
    % circular section, creates platform for parabolic section
    theta = linspace(3*pi/2,7*pi/4,100);
    radius = 10;
    g = 9.81;
    x = radius.*cos(theta) + xI;
    y = theta.*0 + yI;
    z = radius.*sin(theta) + zI + radius;
    vel = sqrt(2 .* g .* (125 - z));
    Gz = ((vel.^2)./(g * radius)) - sin(theta);
    Gx = theta .* 0;
    Gy = theta .* 0;
end
function [x, y, z, vel, Gx, Gy, Gz] = Parabola(v0,pos0)
    %% Setup of variables and vectors
    t = linspace(0,10,100);
    g = 9.81;
    x = ones(1,length(t)).*4200; % filling with large values
    y = ones(1,length(t)).*4200; % to be trimmed later
    z = ones(1,length(t)).*4200;
    x(1) = pos0(1);
    y(1) = pos0(2);
    z(1) = pos0(3);
    i = 2;
    %% Calculating path with kinematics
    while i <= length(t)
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
    vel = sqrt(2*g.*(125-z));
    Gz = t .* 0;
    Gx = t .* 0;
    Gy = t .* 0;
    % z(end) = z(1);
    % vf = sqrt(v0(1) v0(2) -v0(3)];  % output, [m/s]
end

% Transition from hill 
function [x, y, z, vel, Gx, Gy, Gz] = landing(xI,yI,zI)
    % circular section, platform for parabolic landing
    theta = linspace(5*pi/4,3*pi/2,100);
    radius = 10;
    g = 9.81;
    x = radius.*cos(theta) + xI + 7.0711;
    y = theta.*0 + yI;
    z = radius.*sin(theta) + zI + 7.0711;
    vel = sqrt(2 .* g .* (125 - z));
    Gz = ((vel.^2)./(g * radius)) - sin(theta);
    Gx = theta .* 0;
    Gy = theta .* 0;
end
% Valley
function coords = valley(R1,R2,R3)
    th = linspace(pi/2,0,100);  
    th2 = linspace(-pi, 0,100);     
    th3 = linspace(pi, pi/2,100);
    % Vector of the theta throughout the entire valley
    theta = [th,th2,th3];
    
    % First arc (quarter circle)
    a = R1*cos(th)+79.4356;
    b = 50*ones(length(a),1);
    c = R1*sin(th)+73.8100;
    
    % Second arc (semi circle)
    d = R2*cos(th2)+(R1)+(R2)+79.4356;
    e = 50*ones(length(d),1);
    f = R2*sin(th2)+73.8100;
    
    % Third arc (quarter circle)
    g = R3*cos(th3)+(R1)+(2*R2)+R3+79.4356;
    h = 50*ones(length(g),1);
    i = R3*sin(th3)+73.8100;
    
    X = [a,d,g];        % Vector of all X coordinates
    Y = [b',e',h'];     % Vector of all Y coordinates
    Z = [c,f,i];        % Vector of all Z coordinates
    
    gravity = 9.81;
    h_0 = 125;
    velocity = sqrt(2*gravity.*(h_0-Z)); % Velocity throughout the valley
    radius = [R1*ones(length(a),1);R2*ones(length(d),1);...
        R3*ones(length(g),1)];
    
    % The G-force throughout the valley
    G_force = abs(((velocity.^2)./(gravity.*radius))-sin(theta));
    
    coords = [X;Y;Z;theta;velocity;G_force];
end