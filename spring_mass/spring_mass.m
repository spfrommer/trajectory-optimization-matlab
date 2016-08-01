global gridN mass spring damp
gridN = 30;
mass = 10;
spring = 50;
damp = 1;

tic
% Minimize the simulation time
time_min = @(x) (x(1))^2;
% The initial parameter guess; 1 second, fifty lengths, fifty lengthdirs,
% fifty actuated lengths
if exist('optimal','var')
    disp('Using previous solution as starting guess...');
    x0 = optimal;
else
    x0 = [1; ones(gridN, 1) * 0.9; ones(gridN, 1); ones(gridN, 1)];
end
% No linear inequality or equality constraints
A = [];
b = [];
Aeq = [];
Beq = [];
% Lower bound the simulation time at zero seconds, and bound the
% accelerations between -10 and 30
lb = [0;   ones(gridN, 1) * 0; ones(gridN, 1) * -Inf; ones(gridN, 1) * -5];
ub = [Inf; ones(gridN, 1) * 1;  ones(gridN, 1) * Inf;  ones(gridN, 1) * 5];
% Options for fmincon
options = optimoptions(@fmincon, 'TolFun', 0.00000001, 'MaxIter', 10000, ...
                       'MaxFunEvals', 100000, 'Display', 'iter', ...
                       'DiffMinChange', 0.001);
% Solve for the best simulation time + control input
optimal = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, ...
              @spring_mass_constraints, options);

% Discretize the times
sim_time = optimal(1);
delta_time = sim_time / gridN;
times = 0 : delta_time : sim_time - delta_time;
% Get the state + accelerations (control inputs) out of the vector
lengths         = optimal(2             : 1 + gridN);
lengthdirs      = optimal(2 + gridN     : 1 + gridN * 2);
actlengths      = optimal(2 + gridN * 2 : end);

[c, ceq] = spring_mass_constraints(optimal);

% Make the plots
figure();
plot(times, actlengths);
title('Control Input (Actuated Length) vs Time');
xlabel('Time (s)');
ylabel('Actuated Length (m)');
figure();
plot(times, lengthdirs);
title('Length Derivatives vs Time');
xlabel('Time (s)');
ylabel('Length Derivative (m/s)');
figure();
plot(times, lengths);
title('Length vs Time');
xlabel('Time (s)');
ylabel('Length (m)');

disp(sprintf('Finished in %f seconds', toc));