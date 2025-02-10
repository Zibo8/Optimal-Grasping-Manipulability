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
initialize_2(intervals);
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
        if y>get_y_ub(x)
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


%% Creat the vector of obejective value by calculating F_Mean, F_Variance and the Penalty of Distance
% creat the vector of F_Mean
vector_mean_F_total = zeros(number_y,1);       

% creat the vector of F_Variance
vector_var_F_total  = zeros(number_y,1);  

% creat the vector of the penalty
vector_penalty = zeros(number_y,1);

% Creat the vector of obejective value
vector_obj = zeros(number_y,1);

for index=1:number_y
    % calculate mean of Force, ignore NaN
    vector_mean_F_total(index)  = mean(matrix_F_total(index,:),'omitnan');

    % calculate variance of Force, ignore NaN
    vector_var_F_total(index)   = var(matrix_F_total(index,:),'omitnan');

    % calculate penalty of Distance
    vector_penalty(index)  = d_open - get_x_ub(vector_y(index)); 

end

% normalize F_Mean, F_Variance and the Penalty of Distance
normalize_Fmean = normalize(vector_mean_F_total,'range', [-1, 1]);
normalize_Fvar = normalize(vector_var_F_total,'range', [-1, 1]);
normalize_penalty = normalize(vector_penalty,'range', [-1, 1]);
%% calculate objective value
% Set up weight matrix, y* = -57
theta=[-1 1 1];
for index=1:number_y
% calculate objective value
vector_obj(index,1) = theta(1)*normalize_Fmean(index,1) + theta(2)*normalize_Fvar(index,1) + theta(3)*normalize_penalty(index,1);
end

%% find optimal height y by minimizing objective value
[minValue,minIndex]=min(vector_obj);

%% Plot End-effector Force in Cartesian Space

plot_workspace_1(l1,l2);

% Scatter plot for force data
scatter(matrix_x(minIndex,:),matrix_y(minIndex,:), 20, matrix_F_total(minIndex,:) ,'filled')
clim([340 500]);
colorbarLabel = colorbar;
colorbarLabel.Label.String = 'F_{total} (N)';
colorbarLabel.Label.Position = colorbarLabel.Label.Position - [0.35, 0, 0];
