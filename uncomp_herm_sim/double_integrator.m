global gridN
gridN = 10;

tic
% Minimize the simulation time
time_min = @(x) x(1);
% The initial parameter guess; 1 second, fifty positions at nodes,
% fifty velocities at nodes, fifty accelerations at nodes, 
% fifty positions at nodes, fifty velocities at nodes, 
% fifty accelerations at midpoints
x0 = [1; linspace(0,1,gridN)';      linspace(0,1,gridN)'; ... 
         ones(gridN, 1) * 5;        linspace(0,1,gridN - 1)'; ...
         linspace(0,1,gridN - 1)';  ones(gridN - 1, 1) * 5];
% No linear inequality or equality constraints
A = [];
b = [];
Aeq = [];
Beq = [];
% Lower bound the simulation time at zero seconds, and bound the
% accelerations between -10 and 30
lb = [0;    ones(gridN * 2, 1) * -Inf;      ones(gridN, 1) * -10; ...
            ones(gridN * 2 - 2, 1) * -Inf;  ones(gridN - 1, 1) * -10];
ub = [Inf;  ones(gridN * 2, 1) * Inf;       ones(gridN, 1) * 30; ...
            ones(gridN * 2 - 2, 1) * Inf;   ones(gridN - 1, 1) * 30];
% Options for fmincon
options = optimset('TolFun', 0.00000001, 'MaxIter', 100000, ...
                   'MaxFunEvals', 100000);
% Solve for the best simulation time + states + control inputs
optimal = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, ...
              @double_integrator_constraints, options);

% Discretize the times into a vector
sim_time = optimal(1);
delta_time = sim_time / gridN;
% Divide delta time by two to plot against middle states / inputs
times = 0 : delta_time / 2 : sim_time - delta_time;
% Get the states and control inputs out of the optimal vector
nodepositions   = optimal(2             : 1 + gridN);
nodevels        = optimal(2 + gridN     : 1 + gridN * 2);
nodeaccs        = optimal(2 + gridN * 2 : 1 + gridN * 3);
midpositions    = optimal(2 + gridN * 3 : gridN * 4);
midvels         = optimal(1 + gridN * 4 : gridN * 5 - 1);
midaccs         = optimal(gridN * 5 : end);
% Interleave the node and middle positions for display purposes
combpositions = [nodepositions, [midpositions ; 0]]';
combpositions = combpositions(:)';
combpositions = combpositions(1:end-1);
% Interleave the node and middle velocities for display purposes
combvels = [nodevels, [midvels ; 0]]';
combvels = combvels(:)';
combvels = combvels(1:end-1);
% Interleave the node and middle accelerations for display purposes
combaccs = [nodeaccs, [midaccs ; 0]]';
combaccs = combaccs(:)';
combaccs = combaccs(1:end-1);

% Make the plots
figure();
plot(times, combaccs);
title('Control Input (Acceleration) vs Time');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
figure();
plot(times, combvels);
title('Velocity vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
figure();
plot(times, combpositions);
title('Position vs Time');
xlabel('Time (s)');
ylabel('Position (m)');

disp(sprintf('Finished in %f seconds', toc));
