function [ c, ceq ] = double_integrator_constraints( x )
    global gridN
    % No nonlinear inequality constraint needed
    c = [];
    % Calculate the timestep
    sim_time = x(1);
    delta_time = sim_time / gridN;
    % Get the states and control inputs out of the optimal vector
    nodepositions   = x(2             : 1 + gridN);
    nodevels        = x(2 + gridN     : 1 + gridN * 2);
    nodeaccs        = x(2 + gridN * 2 : 1 + gridN * 3);
    midpositions    = x(2 + gridN * 3 : gridN * 4);
    midvels         = x(1 + gridN * 4 : gridN * 5 - 1);
    midaccs         = x(gridN * 5 : end);
    
    % Constrain initial position and velocity to be zero
    ceq = [nodepositions(1); nodevels(1)];
    for i = 1 : length(nodepositions) - 1
        % The state at the beginning of the time interval
        x_i = [nodepositions(i); nodevels(i)];
        % What the state should be at the end of the time interval
        x_n = [nodepositions(i+1); nodevels(i+1)];
        % The state at the middle of the time interval
        x_mid = [midpositions(i); midvels(i)];
        % The time derivative of the state at the beginning of the time
        % interval
        xdot_i = [nodevels(i); nodeaccs(i)];
        % The time derivative of the state at the end of the time interval
        xdot_n = [nodevels(i+1); nodeaccs(i+1)];
        % The time derivative of the state at the middle of the interval
        xdot_mid = [midvels(i); midaccs(i)];
        
        % The interpolated state of the cubic spline at the middle of the
        % time interval
        x_midsp = 0.5 * (x_i + x_n) + (delta_time / 8) * (xdot_i - xdot_n);
        % The interpolated state of the cubic spline at the end of the time
        % interval
        xend = x_i + (delta_time / 6) * (xdot_i + 4 * xdot_mid + xdot_n);
        % Constrain the end state of the cubic spline to be the beginning
        % state of the next time interval
        ceq = [ceq ; x_n - xend; x_mid - x_midsp];
    end
    % Constrain end position to 1 and end velocity to 0
    ceq = [ceq ; nodepositions(end) - 1 ; nodevels(end)];
end