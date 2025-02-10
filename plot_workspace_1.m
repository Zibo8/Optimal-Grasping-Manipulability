function plot_workspace_1(l1,l2)

figure
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

% Define joint angles for arm position
jointAngles = [0, 2];
xJoint2 = l1 * cos(jointAngles(1)); % Joint 2 position
yJoint2 = l1 * sin(jointAngles(1));
xEndEffector = xJoint2 + l2 * cos(jointAngles(1) + jointAngles(2)); % End-effector position
yEndEffector = yJoint2 + l2 * sin(jointAngles(1) + jointAngles(2));

% Plot the robotic arm
plot([0, xJoint2], [0, -yJoint2], 'r', 'LineWidth', 2); % Link 1
plot([xJoint2, xEndEffector], [-yJoint2, -yEndEffector], 'b', 'LineWidth', 2); % Link 2

% Add arrows for boundary limits
quiver([-l1-l2, l1-l2], [0, 0], [5, -5], [0, 0], 'AutoScale', 'off', ...
    'LineWidth', 2, 'LineStyle', ':', 'Color', [0.8, 0.6, 0.2]);

% text
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

end

