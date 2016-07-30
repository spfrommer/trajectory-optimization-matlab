function [ c, ceq ] = double_integrator_constraints( x )
    global gridN
    % No nonlinear inequality constraint needed
    c = [];
    % Calculate the timestep
    sim_time = x(1);
    delta_time = sim_time / gridN;
    % Get the states and control inputs out of the vector
    positions   = x(2             : 1 + gridN);
    vels        = x(2 + gridN     : 1 + gridN * 2);
    nodeaccs    = x(2 + gridN * 2 : 1 + gridN * 3);
    midaccs     = x(2 + gridN * 3 : end);
    
    % Constrain initial position and velocity to be zero
    ceq = [positions(1); vels(1)];
    for i = 1 : length(positions) - 1
        % The state at the beginning of the time interval
        x_i = [positions(i); vels(i)];
        % What the state should be at the end of the time interval
        x_n = [positions(i+1); vels(i+1)];
        % The time derivative of the state at the beginning of the time
        % interval
        xdot_i = [vels(i); nodeaccs(i)];
        % The time derivative of the state at the end of the time interval
        xdot_n = [vels(i+1); nodeaccs(i+1)];
        % The interpolated state of the cubic spline at the middle of the
        % time interval
        x_mid = 0.5 * (x_i + x_n) + (delta_time / 8) * (xdot_i - xdot_n);
        % The time derivative of the interpolated spline at the middle of
        % the time interval
        xdot_m = [x_mid(2); midaccs(i)];
        % The interpolated state of the cubic spline at the end of the time
        % interval
        xend = x_i + (delta_time / 6) * (xdot_i + 4 * xdot_m + xdot_n);
        % Constrain the end state of the cubic spline to be the beginning
        % state of the next time interval
        ceq = [ceq ; x_n - xend];
    end
    % Constrain end position to 1 and end velocity to 0
    ceq = [ceq ; positions(end) - 1 ; vels(end)];
end