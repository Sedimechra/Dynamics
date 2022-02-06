function [loop,vf,G] = loopPath(pos0,r,prop)
    %% Setup
    % Note, this currently only calculated velocity correctly
    % if the path is one full loop. I need to add the calculations
    % for the velocity vector for a partial loop.
    h0 = 125; % [m] initial height
    m = 100;  % [kg] particle mass
    g = 9.81; % [m/s^2] gravity
    t = linspace(-pi/2,(prop*2*pi)-pi/2,1000);
    %% calculating positions
    x = pos0(1) + r*cos(t);
    z = pos0(3) + r + r*sin(t);
    y = pos0(2) + 0*t;         % If you change t to be multiplied by a nonzero number it becomes a helix
    %% velocity and G force calculations
    V = sqrt(2*g*(h0-z));
    N = m*((cos(t)*g)+((V.^2)/r));
    G = N/(m*g);
    %% output
    vf = V(end);
    loop = [x; y; z]; %[meters]
end