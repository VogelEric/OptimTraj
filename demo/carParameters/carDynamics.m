function xdot = carDynamics(t,x,u,p)
    %CARDYNAMICS Summary of this function goes here
    %   Detailed explanation goes here
    
    % p = parameter to optimize.
    
    mass = p(1);
    
    % A larger engine has a larger acceleration, but has more mass.
    % The larger engine has diminishing returns with mass^(1/2)
    maxAccel = 2500*(mass-999)^(1/2);
    
    idx = u>0; % if accelerating
    u(idx) = u(idx)*maxAccel;
    
    % braking
    u(~idx) = u(~idx)*25000;
    xdot = [x(2,:);u/mass];
end
