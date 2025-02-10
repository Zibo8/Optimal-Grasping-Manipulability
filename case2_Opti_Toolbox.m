%% Initialize
% l1:       the length of robot arm_1,  the unit is cm
% l2:       the length of robot arm_1,  the unit is cm

% d_close:  robot arm fully closed,   the unit is cm 
% d_open:   robot arm fully open,     the unit is cm

% torque:   torque of Joint, the unit is N*cm, clockwise direction is negative

% vector_x, x is samplede in [-28 32]
% vector_y, y is samplede in [-65.4492 0]
% Sampling intervals, the unit is cm
intervals = 0.1;

% Friction Coefficient
mu = 0.6;

[l1,l2,d_close,d_open,torque,vector_x,vector_y,number_x,number_y] = ...
initialize_1(intervals);                
 
%% find minimum

% first row,  store x at minimal F_x
% second row, store y at minimal F_x
% third row,  store the minimal F_x
% fourth row, store q1 at minimal F_x
% third row,  store q2 at minimal F_x
atMinFx_xyFxQ = zeros(5,number_x);

index = 1;

for x = vector_x

    %store x
    atMinFx_xyFxQ(1, index) = x;            

    % objective Function
    objective = @(y) computeForce(l1, l2, x, y, torque,mu);
    
    % numbers of variable for objective Function
    nvars=1;
    
    % lower boundary and upper boundart of y
    lb = get_y_lb(x);   
    ub = get_y_ub(x);   

    % Method_1: Particle Swarm Optimization
    options = optimoptions('particleswarm', ...
                       'Display', 'iter', ...         % Display information of iteratino
                       'SwarmSize', 50, ...           % Numbers of swarm
                       'MaxIterations', 200);         % Max Iterations
    [y_optimal, fval] = particleswarm(objective, nvars, lb, ub, options);

%     % Method_2: Genetic Algorithm
%     options = optimoptions('ga', 'Display', 'iter');
%     [y_optimal, fval] = ga(objective, nvars, [], [], [], [], lb, ub, [], options);

    % store y at minimal force
    atMinFx_xyFxQ(2, index) = y_optimal;      
    % store F at minimal force
    atMinFx_xyFxQ(3, index) = fval;       

    index = index + 1;
end
%% Plot trajectory at max F_total in Car
plot_workspace_1(l1,l2);

% Scatter plot for force data
scatter(atMaxFtotal_xyFQ(1,:),atMaxFtotal_xyFQ(2,:), 20, atMaxFtotal_xyFQ(3,:) ,'filled')
clim([0 1000]);
colorbarLabel = colorbar;
colorbarLabel.Label.String = 'F_{total} (N)';
colorbarLabel.Label.Position = colorbarLabel.Label.Position - [0.35, 0, 0];

%% function

function total_force = computeForce(l1, l2, x, y, torque,mu)
% inverse kinematics to calculate q
q = inv_kinematics(l1, l2, x, y);

% calculate inverse of transform Jocobian Matrix
JStar = get_JStar(l1, l2, q(1), q(2));

% calculate Horizontal_force
Horizontal_force = torque * (JStar(1,1) + JStar(1,2));

% calculate End-effector Force in the vertical direction
vertical_force = torque * (JStar(2,1) + JStar(2,2));

% calculate End-effector Force in the vertical direction
% Friction Force is Positive, so (-Horizontal_force * mu) is used
total_force = -Horizontal_force * mu + vertical_force;
end
