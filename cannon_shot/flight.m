% The function to be minimized (E = vx^2 + vy^2)
energy_fun = @(x) (x(2)^2 + x(3)^2);

% The initial parameter guess; time = 1; vx_0 = 2, vy_0 = 3
x0 = [1; 2; 3];
% No linear inequality or equality constraints
A = [];
b = [];
Aeq = [];
Beq = [];
% Lower bound the simulation time at zero, leave velocities unbounded
lb = [0; -Inf; -Inf];
ub = [Inf; Inf; Inf];
% Solve for the best simulation time + control input
optimal = fmincon(energy_fun, x0, A, b, Aeq, Beq, lb, ub, ...
                  @flight_constraint, options);
% Unpack the parameters
opt_time = optimal(1);
opt_vx = optimal(2);
opt_vy = optimal(3);

fprintf('Optimal time: %f\n', opt_time);
fprintf('Optimal initial velocity: [%f, %f]\n', opt_vx, opt_vy);