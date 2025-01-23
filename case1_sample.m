%% Initialize

l1 = 32.3097;       % length of robot arm_1, the unit is cm
l2 = 38.1395;       % length of robot arm_2, the unit is cm

d_close = -28;      % robot arm fully closed, the unit is cm            
d_open  =  32;      % robot arm fully open,   the unit is cm  

torque =  10000;    % torque of Joint, the unit is N*cm

% sample, sampling intervals = 0.1 cm
% l1 + l2 -5 = 65.4492 cm
vector_x = -28:0.1:32;            
vector_y = -(0:0.1:65.4492)';

% determin numer_x and number_y based on vector_x and vector_y
number_x = 601;                    
number_y = 655;

%% Crear x_matrix and y_matrix, set date outside the workspace to NaN(not a number)
% Crear x_matrix and y_matrix
x_matrix = repmat(vector_x,number_y,1);
y_matrix = repmat(vector_y,1,number_x);

% if date outside the workspace, then set to NaN
i = 1;
for x = vector_x
    j = 1;
    for y = (vector_y)'
        if y>get_y_ub(x) || y<get_y_lb(x)
            x_matrix(j,i) = NaN;
            y_matrix(j,i) = NaN;
        end
        j = j + 1;
    end
    i = i + 1;
end

%% Calculate q_1, q_2 and Force based on x_matrix and y_matrix
q1_matrix = zeros(number_y,number_x);
q2_matrix = zeros(number_y,number_x);
F_matrix = zeros(number_y,number_x);

for j=1:1:number_x
    for i=1:1:number_y
        % inverse kinematics to calculate q
        q = inv_kinematics (l1, l2, x_matrix(i,j), y_matrix(i,j));
        % store q_1
        q1_matrix(i,j) = q(1);
        % store q_2
        q2_matrix(i,j) = q(2);
        % calculate inverse of transform Jocobian Matrix
        JStar = get_JStar(l1,l2,q(1),q(2));
        % calculate Horizontal_force
        Horizontal_force = -torque * (JStar(1,1) + JStar(1,2));
        % store Horizontal_force
        F_matrix(i,j) = Horizontal_force; 

    end
end

%% find minimum Force (force is negative)

% first row,  store x at minimal force
% second row, store y at minimal force
% third row,  store minimal force
store_xyF_atMin = zeros(3,number_x);

% store q_1 and q_2 at minimal force
store_Q1_atMin  = zeros(1,number_x);
store_Q2_atMin  = zeros(1,number_x);

for index = 1:number_x
    % find mininum of Force
    [min_value, min_index] = min(F_matrix(:, index));   

    % store x at minimal force
    store_xyF_atMin(1,index) = x_matrix(min_index,index);
    % store y at minimal force
    store_xyF_atMin(2,index) = y_matrix(min_index,index);
    % store F at minimal force
    store_xyF_atMin(3,index) = min_value;                 

    store_Q1_atMin(1,index)  = q1_matrix(min_index,index);
    store_Q2_atMin(1,index)  = q2_matrix(min_index,index);
end

%% Plot 
figure;
hold on;
grid on;

% Define end-effector positions for workspace boundaries
xOuterArc = linspace(-l1-l2+5, l1+l2-5, 100);
yOuterArc = sqrt((l1 + l2 - 5)^2 - xOuterArc.^2); % Green outer arc
xInnerArc = linspace(l1-l2-5, -(l1-l2-5), 100);
yInnerArc = sqrt((l1 - l2 - 5)^2 - xInnerArc.^2); % Green inner arc
xRedArcInner = linspace(l1-l2, -(l1-l2), 100);
yRedArcInner = sqrt((l1 - l2)^2 - xRedArcInner.^2); % Red inner arc
xBlackArc = linspace(l1-l2, l1+l2, 100);
yBlackArc = sqrt(l2^2 - (xBlackArc - l1).^2); % Black arc
xFullArc = linspace(-l1-l2, l1+l2, 100);
yFullArc = sqrt((l1 + l2)^2 - xFullArc.^2); % Full outer arc

% Define fill area for safe workspace
xFillOuter = linspace(-28, 32, 100);
yFillOuter = sqrt((l1 + l2 - 5)^2 - xFillOuter.^2);
xFillInnerGreen = linspace(l1-l2-5, -4.51, 100);
yFillInnerGreen = sqrt((l1 - l2 - 5)^2 - xFillInnerGreen.^2);
xFillInnerBlack = linspace(-4.51, 32, 100);
yFillInnerBlack = sqrt(l2^2 - (xFillInnerBlack - l1).^2);

% Fill vertical boundary lines
xFillLeft = [-28, -28];
yFillLeft = [0, sqrt((l1 + l2 - 5)^2 - 28^2)];
xFillRight = [32, 32];
yFillRight = [sqrt((l1 + l2 - 5)^2 - 32^2), sqrt(l2^2 - (32 - l1).^2)];

% Fill safe workspace with light gray
fill([xFillLeft, xFillOuter, fliplr(xFillInnerBlack), fliplr(xFillInnerGreen)], ...
     [-yFillLeft, -yFillOuter, -fliplr(yFillInnerBlack), -fliplr(yFillInnerGreen)], ...
     [0.9, 0.9, 0.9], 'EdgeColor', 'none');

% Plot workspace boundaries
plot(xOuterArc, -yOuterArc, 'g--', 'LineWidth', 1); % Green outer boundary
plot(xInnerArc, -yInnerArc, 'g--', 'LineWidth', 1); % Green inner boundary
plot(xRedArcInner, -yRedArcInner, 'r--', 'LineWidth', 1); % Red inner boundary
plot(xFullArc, -yFullArc, 'r--', 'LineWidth', 1); % Full outer boundary
plot(xBlackArc, -yBlackArc, 'k--'); % Black arc
plot([-l1-l2, l1-l2-5], [0, 0], 'k--'); % Horizontal boundary
plot(xFillLeft, -yFillLeft, 'k--', xFillRight, -yFillRight, 'k--'); % Vertical boundaries

% Plot base point
plot(0, 0, '.', 'MarkerSize', 20, 'Color', 'k'); 

% Define joint angles for arm position (example values)
jointAngles = [0, 2]; % Replace with actual joint angles
xJoint1 = l1 * cos(jointAngles(1)); % Joint 1 position
yJoint1 = l1 * sin(jointAngles(1));
xEndEffector = xJoint1 + l2 * cos(jointAngles(1) + jointAngles(2)); % End-effector position
yEndEffector = yJoint1 + l2 * sin(jointAngles(1) + jointAngles(2));

% Plot the robotic arm
plot([0, xJoint1], [0, -yJoint1], 'r', 'LineWidth', 2); % Link 1
plot([xJoint1, xEndEffector], [-yJoint1, -yEndEffector], 'b', 'LineWidth', 2); % Link 2

% Add arrows for boundary limits
quiver([-l1-l2, l1-l2], [0, 0], [5, -5], [0, 0], 'AutoScale', 'off', ...
       'LineWidth', 2, 'LineStyle', ':', 'Color', [0.8, 0.6, 0.2]);

% Scatter plot for force data (example, replace store_xyF_atMin with actual data)
scatter(store_xyF_atMin(1, :), store_xyF_atMin(2, :), 20, store_xyF_atMin(3, :), 'filled');
colorbarLabel = colorbar;
colorbarLabel.Label.String = 'F (N)';

% Annotate the workspace
text(-20, -55, 'Safe Workspace', 'FontSize', 12, 'FontWeight', 'bold');
text(-60, -72, 'x = -28cm, fully closed', 'FontSize', 12, 'FontWeight', 'bold');
text(20, -72, 'x = 32cm, fully open', 'FontSize', 12, 'FontWeight', 'bold');
text(-70, 5, '5cm', 'FontWeight', 'bold', 'FontSize', 12);
text(-15, 5, '5cm', 'FontWeight', 'bold', 'FontSize', 12);
text(0, 6, 'Base Point', 'FontSize', 12, 'FontWeight', 'bold');

% Set axis and labels
set(gca, 'FontSize', 14);
axis([-l1-l2-10, l1+l2+10, -90, 30]);
xlabel('x (cm)');
ylabel('y (cm)');