function [theta_exp,w_exp,v_exp,time] = LCSDATA(fileName)
    % Reading in the file, interpreting columns
    data = readtable(fileName);
    data.Properties.VariableNames = ["Time","Theta","yDist","AngVel","SlideSpeed","SampleT"];
    % converting slide speed to cm/s
    data.SlideSpeed = data.SlideSpeed./10;
    % subtacting out extra rotations
    data.Theta = data.Theta - (floor((data.Theta(1,1)-152.5)/360)*360);
    % determining the total amount of turns
    data.totalTurns = (data.Theta - data.Theta(1,1))./360;
    % finding where the disk completes 6 rotations
    [~,ind] = min(abs(data.totalTurns-6));
    % getting rid of all data beyond 6 rotations
    data([ind+1:end],:) = [];
    time = data.Time(:,1);
    theta_exp = data.Theta;
    w_exp = deg2rad(data.AngVel);
    v_exp = data.SlideSpeed;
end