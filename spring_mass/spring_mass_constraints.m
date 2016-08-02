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
    actlengths      = x(2 + gridN * 2 : 1 + gridN * 3);
    actlengthdirs   = x(2 + gridN * 3 : 1 + gridN * 4);
    actlengthddirs  = x(2 + gridN * 4 : end);
    
    ceq = ones(4 * (gridN - 1), 1);
    for i = 1 : gridN-1
        % The state at the beginning of the time interval
        x_i = [lengths(i); lengthdirs(i); actlengths(i); actlengthdirs(i)];
        % What the state should be at the end of the time interval
        x_n = [lengths(i+1); lengthdirs(i+1); ...
               actlengths(i+1); actlengthdirs(i+1)];
        
        A = [0 1 0 0;
            -spring/mass -damp/mass spring/mass damp/mass;
             0 0 0 1;
             0 0 0 0];
        Bi = [0; 0; 0; actlengthddirs(i)];
        Bn = [0; 0; 0; actlengthddirs(i+1)];
        
        % The time derivative of the state at the beginning of the time
        % interval
        xdot_i = A * x_i + Bi;
        % The time derivative of the state at the end of the time interval
        xdot_n = A * x_n + Bn;
        % The end position of the time interval calculated using quadrature
        xend = x_i + delta_time * (xdot_i + xdot_n) / 2;
        % Constrain the end state of the current time interval to be
        % equal to the starting state of the next time interval
        ceq((i-1)*4+1: i*4) = x_n - xend;
    end
    % Constrain initial position to be one and velocity to be zero
    ceq = [ceq; lengths(1) - 1; lengthdirs(1); actlengths(1) - 1; actlengthdirs(i)];
    % Constrain end position to 1 and end velocity to 0
    ceq = [ceq; lengths(end) - 0.8];
end