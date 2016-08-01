function [ c, ceq ] = spring_mass_constraints( x )
    global gridN mass spring damp
    % No nonlinear inequality constraint needed
    c = [];
    % Calculate the timestep
    sim_time = x(1);
    delta_time = sim_time / gridN;
    % Get the states / inputs out of the vector
    lengths         = x(2             : 1 + gridN);
    lengthdirs      = x(2 + gridN     : 1 + gridN * 2);
    actlengths      = x(2 + gridN * 2 : end);
    
    % Constrain initial position to be one and velocity to be zero
    ceq = [lengths(1) - 1; lengthdirs(1)];
    for i = 1 : gridN-1
        % The state at the beginning of the time interval
        x_i = [lengths(i); lengthdirs(i)];
        % What the state should be at the end of the time interval
        x_n = [lengths(i+1); lengthdirs(i+1)];
        
        A = [0 1; -spring/mass -damp/mass];
        B = [0; actlengths(i)*spring/mass];
        
        xdot_i = A * x_i + B;
        ceq = [ceq ; x_n - (x_i + delta_time * xdot_i)];
        % The time derivative of the state at the beginning of the time
        % interval
        % xdot_i = [lengthdirs(i); -spring(i)];
        % The time derivative of the state at the end of the time interval
        %xdot_n = [vels(i+1); accs(i+1)];
        
        % The end position of the time interval calculated using quadrature
        %xend = x_i + delta_time * (xdot_i + xdot_n) / 2;
        % Constrain the end position of the current time interval to be
        % equal to the starting position of the next time interval
        %ceq = [ceq ; x_n - xend];
    end
    % Constrain end position to 1 and end velocity to 0
    ceq = [ceq ; lengths(end) - 0.8; lengthdirs(end)];
end