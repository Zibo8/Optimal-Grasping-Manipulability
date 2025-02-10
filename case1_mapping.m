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
mu = 0.86;

[l1,l2,d_close,d_open,torque,vector_x,vector_y,number_x,number_y] = ...
initialize_1(intervals);


%% Crear matrix_x and matrix_y, set date outside the workspace to NaN(not a number)
% Crear matrix_x and matrix_y
matrix_x = repmat(vector_x,number_y,1);
matrix_y = repmat(vector_y,1,number_x);

% We record NaN to aviod that other calculations may cause NaN
record_NaN=zeros(number_y,number_x);

% if data outside the workspace, then set to NaN
j = 1;
for x = vector_x
    i = 1;
    for y = (vector_y)'
        if y>get_y_ub(x) || y<get_y_lb(x)
            matrix_x(i,j) = NaN;
            matrix_y(i,j) = NaN;
            record_NaN(i,j)=NaN;
        end
        i = i + 1;
    end
    j = j + 1;
end

%% Calculate q_1, q_2 and Force based on x_matrix and y_matrix
% Crear matrix_q1, matrix_q2 and matrix_F
matrix_q1 = zeros(number_y,number_x);
matrix_q2 = zeros(number_y,number_x);

% End-effector Force in the horizontal direction
matrix_F_x = zeros(number_y,number_x);
matrix_F_x_pos = zeros(number_y,number_x);

% End-effector Force in the vertical direction
matrix_F_y = zeros(number_y,number_x);

% Friction Force + End-effector Force in the vertical direction
matrix_F_total = zeros(number_y,number_x);

for j=1:1:number_x
    for i=1:1:number_y

        % inverse kinematics to calculate q
        q = inv_kinematics (l1, l2, matrix_x(i,j), matrix_y(i,j));

        % store q_1, the unit is radian
        matrix_q1(i,j) = q(1);

        % store q_2, the unit is radian
        matrix_q2(i,j) = q(2);

        % calculate inverse of transform Jocobian Matrix
        JStar = get_JStar(l1,l2,q(1),q(2));

        % calculate End-effector Force in the vertical direction and store
        Horizontal_force = torque * (JStar(1,1) + JStar(1,2));
        
        if Horizontal_force<0
            matrix_F_x(i,j) = Horizontal_force;

            % calculate End-effector Force in the vertical direction and store
            vertical_force = torque * (JStar(2,1) + JStar(2,2));
            matrix_F_y(i,j) = vertical_force;

            % calculate End-effector Force in the vertical direction and store
            % Friction Force is Positive, so (-Horizontal_force * mu) is used
            total_force = -Horizontal_force * mu + vertical_force;
            matrix_F_total(i,j) = total_force;
        else
            record_NaN(i,j)=NaN;
            matrix_F_x(i,j) = NaN;
            matrix_F_x_pos(i,j) = Horizontal_force;
            matrix_F_y(i,j) = NaN;
            matrix_F_total(i,j) = NaN;
        end
    end
end

%% For plotting, the matrix shape is changed
% new_shape_x = reshape(matrix_x,1,[]);
% new_shape_y = reshape(matrix_y,1,[]);
% 
% new_shape_F_x = reshape(matrix_F_x,1,[]);
% new_shape_F_x_pos = reshape(matrix_F_x_pos,1,[]);
% new_shape_F_y = reshape(matrix_F_y,1,[]);
% new_shape_F_total = reshape(matrix_F_total,1,[]);
% 
% new_shape_q1 = reshape(matrix_q1,1,[]);
% new_shape_q2 = reshape(matrix_q2,1,[]);
%% Plot joint space
% figure
% grid on
% hold on
% 
% % plot Force in joint space
% scatter(new_shape_q1, new_shape_q2, 20, [0.9 0.9 0.9], 'filled');
% 
% % Set axis and labels
% xlabel('q_1 (radian)')
% ylabel('q_2 (radian)')
% axis([-2 1 -3.5 -0.5]);
% set(gca,'FontSize',14);

%% F_x > 0 
% plot_workspace_1(l1,l2);
% 
% % Scatter plot for force data
% scatter(new_shape_x,new_shape_y, 20, new_shape_F_x_pos ,'filled')
% colorbarLabel = colorbar;
% colorbarLabel.Label.String = 'F_{x} (N)';
% colorbarLabel.Label.Position = colorbarLabel.Label.Position - [0.35, 0, 0];

%% Plot End-effector Force in the horizontal direction in Cartesian Space

% plot_workspace_1(l1,l2);
% 
% % Scatter plot for force data
% scatter(new_shape_x,new_shape_y, 20, new_shape_F_x ,'filled')
% clim([-1000 0]);
% colorbarLabel = colorbar;
% colorbarLabel.Label.String = 'F_{x} (N)';
% colorbarLabel.Label.Position = colorbarLabel.Label.Position - [0.35, 0, 0];


%% Plot End-effector Force in the vertical direction in Cartesian Space

% plot_workspace_1(l1,l2);
% 
% % Scatter plot for force data
% scatter(new_shape_x,new_shape_y, 20, new_shape_F_y ,'filled')
% clim([0 1000]);
% colorbarLabel = colorbar;
% colorbarLabel.Label.String = 'F_{y} (N)';
% colorbarLabel.Label.Position = colorbarLabel.Label.Position - [0.35, 0, 0];


%% Plot End-effector Force in the horizontal direction in joint space
% figure
% grid on
% hold on
% 
% % plot Force in joint space
% scatter(new_shape_q1,new_shape_q2, 20, new_shape_F_x ,'filled')
% clim([-1000 0]);
% colorbarLabel = colorbar;
% colorbarLabel.Label.String = 'F_{x} (N)';
% colorbarLabel.Label.Position = colorbarLabel.Label.Position - [0.15, 0, 0];
% 
% % Set axis and labels
% xlabel('q_1 (radian)')
% ylabel('q_2 (radian)')
% axis([-2 1 -3.5 -0.5]);
% set(gca,'FontSize',14);

%% Plot End-effector Force in the vertical direction in joint space
% figure
% grid on
% hold on
% 
% % plot Force in joint space
% scatter(new_shape_q1,new_shape_q2, 20, new_shape_F_y ,'filled')
% clim([0 1000]);
% colorbarLabel = colorbar;
% colorbarLabel.Label.String = 'F_{y} (N)';
% colorbarLabel.Label.Position = colorbarLabel.Label.Position - [0.15, 0, 0];
% 
% % Set axis and labels
% xlabel('q_1 (radian)')
% ylabel('q_2 (radian)')
% axis([-2 1 -3.5 -0.5]);
% set(gca,'FontSize',14);
