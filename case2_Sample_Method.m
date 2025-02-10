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
            matrix_F_y(i,j) = NaN;
            matrix_F_total(i,j) = NaN;
        end
    end
end

%% Find minimum(negative)/maximum(positive)
% Find minimum Force in the horizontal direction
% Find maximum Force in the vertical direction
% Find maximum total Force

% first row,  store x at minimal F_x
% second row, store y at minimal F_x
% third row,  store the minimal F_x
% fourth row, store q1 at minimal F_x
% third row,  store q2 at minimal F_x
atMinFx_xyFxQ = zeros(5,number_x);

% first row,  store x at max F_y
% second row, store y at max F_y
% third row,  store the max F_y
% fourth row, store q1 at max F_y
% third row,  store q2 at max F_y
atMaxFy_xyFyQ = zeros(5,number_x);

% first row,  store x at max F_total
% second row, store y at max F_total
% third row,  store the max F_total
% fourth row, store q1 at max F_total
% third row,  store q2 at max F_total
atMaxFtotal_xyFQ = zeros(5,number_x);

% first row,  store F_x at max F_total
% second row, store F_y at max F_total
atMaxFtotal_FxFy = zeros(2,number_x);

for index = 1:number_x
    % find mininum F_x and store data
    [min_value, min_index] = min(matrix_F_x(:, index));
    atMinFx_xyFxQ(1,index)=matrix_x(min_index,index);
    atMinFx_xyFxQ(2,index)=matrix_y(min_index,index);
    atMinFx_xyFxQ(3,index)=min_value;
    atMinFx_xyFxQ(4,index)=matrix_q1(min_index,index);
    atMinFx_xyFxQ(5,index)=matrix_q2(min_index,index);

    % find max F_y and store data
    [max_value_1, max_index_1] = max(matrix_F_y(:, index));
    atMaxFy_xyFyQ(1,index)=matrix_x(max_index_1,index);
    atMaxFy_xyFyQ(2,index)=matrix_y(max_index_1,index);
    atMaxFy_xyFyQ(3,index)=max_value_1;
    atMaxFy_xyFyQ(4,index)=matrix_q1(max_index_1,index);
    atMaxFy_xyFyQ(5,index)=matrix_q2(max_index_1,index);
   
     % find max F_total and store data
    [max_value_2, max_index_2] = max(matrix_F_total(:, index));
    atMaxFtotal_xyFQ(1,index)=matrix_x(max_index_2,index);
    atMaxFtotal_xyFQ(2,index)=matrix_y(max_index_2,index);
    atMaxFtotal_xyFQ(3,index)=max_value_2;
    atMaxFtotal_xyFQ(4,index)=matrix_q1(max_index_2,index);
    atMaxFtotal_xyFQ(5,index)=matrix_q2(max_index_2,index);

    atMaxFtotal_FxFy(1,index)= matrix_F_x(max_index_2,index);
    atMaxFtotal_FxFy(2,index)= matrix_F_y(max_index_2,index);
end

%% Plot trajectory at max F_total in Car
plot_workspace_1(l1,l2);

% Scatter plot for force data
scatter(atMaxFtotal_xyFQ(1,:),atMaxFtotal_xyFQ(2,:), 20, atMaxFtotal_xyFQ(3,:) ,'filled')
clim([0 1000]);
colorbarLabel = colorbar;
colorbarLabel.Label.String = 'F_{total} (N)';
colorbarLabel.Label.Position = colorbarLabel.Label.Position - [0.35, 0, 0];

%% Plot trajectory at max F_total in Joint Space

% plot_jointSpace_1(matrix_q1,matrix_q2);
% 
% scatter(atMaxFtotal_xyFQ(4,:),atMaxFtotal_xyFQ(5,:), 20, atMaxFtotal_xyFQ(3,:) ,'filled')
% clim([0 1000]);
% colorbarLabel = colorbar;
% colorbarLabel.Label.String = 'F_{total} (N)';
% colorbarLabel.Label.Position = colorbarLabel.Label.Position - [0.35, 0, 0];
