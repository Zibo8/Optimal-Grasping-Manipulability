%% init
% Assump: Robot is in first and second quadrant

% q                 % 单位：Radian(角度)
l1 = 32.3097;       % 单位：cm
l2 = 38.1395;       % 单位：cm
r_max_safe = l1 + l2 -5;

d_close = -28;      % 单位：cm      % x blong to [d_close,d_open]        
d_open  =  32;

torque =  10000;    % 单位：N*cm = 100 N*m

vec_signle_x = -28:0.1:32;     % 采样矩阵和采样密度,确定num_x and num_y
vec_single_y = (0:0.1:r_max_safe)';
num_x = 601;                    % 采样矩阵和采样密度
num_y = 655;

%% 构造完整矩阵
store0_x = repmat(vec_signle_x,num_y,1);
store0_y = repmat(vec_single_y,1,num_x);
store0_q1 = zeros(num_y,num_x);
store0_q2 = zeros(num_y,num_x);
store0_F = zeros(num_y,num_x);

index1 = 1;
for x = vec_signle_x
    index2 = 1;
    for y = (vec_single_y)'
        q = inv_kine (l1, l2, x, y);         % IK
        store0_q1(index2,index1) = q(1);     % store q
        store0_q2(index2,index1) = q(2);     % store q
        
        JStar = get_JStar(l1,l2,q(1),q(2));  % J_Star
        Horizontal_force = torque * (JStar(1,1) + JStar(1,2));    % Horizontal_force

        store0_F(index2,index1) = Horizontal_force;               % store F

        index2 = index2 + 1;
    end

    index1 = index1 + 1;
end
%% 根据SafeWS, 裁剪矩阵__Case 1
store1_x = store0_x;
store1_y = store0_y;
store1_q1 = store0_q1;
store1_q2 = store0_q2;
store1_F = store0_F;

index1 = 1;
for x = vec_signle_x
    index2 = 1;
    for y = (vec_single_y)'
        if y<get_yBoundary1(l1,l2,x) || y>get_yBoundary2(l1,l2,x)
            store1_x(index2,index1) = NaN;
            store1_y(index2,index1) = NaN;
            store1_q1(index2,index1) = NaN;
            store1_q2(index2,index1) = NaN;
            store1_F(index2,index1) = NaN;
        end
        index2 = index2 + 1;
    end
    index1 = index1 + 1;
end

%% case1: find minimum
store_minXYF = zeros(3,num_x);
store_minQ1  = zeros(1,num_x);
store_minQ2  = zeros(1,num_x);

% index是纵索引，min_index是横索引
for index = 1:num_x

    [min_value, min_index] = min(store1_F(:, index));   % mininum of Force

    store_minXYF(1,index) = store1_x(min_index,index);  % store x
    store_minXYF(2,index) = store1_y(min_index,index);  % store y
    store_minXYF(3,index) = min_value;                  % store force

    store_minQ1(1,index)  = store1_q1(min_index,index);
    store_minQ2(1,index)  = store1_q2(min_index,index);
end

%% 10 条竖直线，线上有力（scatter），有workspace, 标注最小力的位置
figure;
hold on;
grid on;
xEnd1 = linspace(-l1-l2+5,l1+l2-5,100);
yEnd1 = sqrt((l1 + l2 - 5 )^2 - xEnd1.^2);          % 绿色圆弧（外侧
xEnd2 = linspace(l1-l2-5,-(l1-l2-5),100);       
yEnd2 = sqrt((l1 - l2-5)^2 - xEnd2.^2);             % 绿色圆弧 (右侧
xEnd3 = linspace(l1-l2,-(l1-l2),100);
yEnd3 = sqrt((l1 - l2)^2 - xEnd3.^2);               % 红色圆弧（内侧
xEnd4 = linspace(l1-l2, l1+l2, 100);                % 黑色圆弧 (右侧
yEnd4 = sqrt(l2^2 - (xEnd4 - l1).^2); 
xEnd = linspace(-l1-l2,l1+l2,100);
yEnd = sqrt((l1 + l2)^2 - xEnd.^2);                 % 红色圆弧(外侧

xEnd1fill= linspace(-28,32,100);
yEnd1fill = sqrt((l1 + l2 - 5 )^2 - xEnd1fill.^2);    % 绿色圆弧（外侧
xEnd2fill = linspace(l1-l2-5,-4.51,100);       
yEnd2fill = sqrt((l1 - l2-5)^2 - xEnd2fill.^2);       % 绿色圆弧 (右侧
xEnd4fill = linspace(-4.51, 32, 100);                 % 黑色圆弧 (右侧
yEnd4fill = sqrt(l2^2 - (xEnd4fill - l1).^2); 
xFillVerLeft =[-28 -28];                              % x=-28
yFillVerLeft =[0 sqrt((l1 + l2 - 5 )^2 - 28^2)];      
xFillVerRight = [32 32];                              % x= 32
yFillVerRight = [sqrt((l1 + l2 - 5 )^2 - 32^2) sqrt(l2^2 - (32 - l1).^2) ];

fill([xFillVerLeft,xEnd1fill,fliplr(xEnd4fill),fliplr(xEnd2fill)] ,[-yFillVerLeft,-yEnd1fill,-fliplr(yEnd4fill),-fliplr(yEnd2fill)], [0.9, 0.9, 0.9], 'EdgeColor', 'none'); % 填充上半圆区域为浅灰色

plot(xEnd1, -yEnd1 , 'g--','LineWidth',1);             % Safe workspace
plot(xEnd2, -yEnd2 , 'g--','LineWidth',1);             % Safe workspace
plot(xEnd3, -yEnd3 , 'r--','LineWidth', 1);            % 红色圆弧（内
plot(xEnd, -yEnd , 'r--','LineWidth',1);               % 红色/黑色圆弧（外侧    
plot(xEnd4,-yEnd4,'k--');    % 黑色圆弧 (右侧
plot([-l1-l2, l1-l2-5], [0, 0], 'k--');               % 工作空间的水平边界
plot(xFillVerLeft,-yFillVerLeft,'k--',xFillVerRight,-yFillVerRight,'k--')
plot(0, 0, '.', 'MarkerSize', 20, 'Color', 'k');    % plot base point

q = [0,2];
x1 = l1 * cos(q(1));                   % 第一个关节位置
y1 = l1 * sin(q(1));
x2 = x1 + l2 * cos(q(1) + q(2));       % 末端执行器位置
y2= y1 + l2 * sin(q(1) + q(2));
plot([x1, x2], [-y1, -y2], 'b', 'LineWidth', 2);      % 绘制第二段机械臂
plot([0, x1], [0, -y1], 'r', 'LineWidth', 2);        % 绘制第一段机械臂

% arrow
quiver([-l1-l2 l1-l2],[0 0],[5 -5],[0 0],'AutoScale', 'off','LineWidth',2,'LineStyle',':','Color',[0.8, 0.6, 0.2]);

% 2D_XYF,在minF
for i=1:50:601
    scatter(store1_x(:,i),-store1_y(:,i),20,store1_F(:,i),'filled');
end

cb=colorbar;
cb.Label.String = 'F (N)';

% 标注工作空间
text(-60,-72,'x = -28cm,fully closed','FontSize',12,'FontWeight', 'bold')
text(20,-72,'x = 32cm,fully open','FontSize',12,'FontWeight', 'bold')
text(-70,5,'5cm','FontWeight', 'bold','FontSize',12)
text(-15,5,'5cm','FontWeight', 'bold','FontSize',12)
text(0,6,'Base Point','FontSize',12,'FontWeight', 'bold')

set(gca,'FontSize',14)
axis([-l1-l2-10 l1+l2+10 -90 30]);
xlabel('x(cm)')
ylabel('y(cm)')

%% 2D_XYminF,并有workspace
figure;
hold on;
grid on;
xEnd1 = linspace(-l1-l2+5,l1+l2-5,100);
yEnd1 = sqrt((l1 + l2 - 5 )^2 - xEnd1.^2);          % 绿色圆弧（外侧
xEnd2 = linspace(l1-l2-5,-(l1-l2-5),100);       
yEnd2 = sqrt((l1 - l2-5)^2 - xEnd2.^2);             % 绿色圆弧 (右侧
xEnd3 = linspace(l1-l2,-(l1-l2),100);
yEnd3 = sqrt((l1 - l2)^2 - xEnd3.^2);               % 红色圆弧（内侧
xEnd4 = linspace(l1-l2, l1+l2, 100);                % 黑色圆弧 (右侧
yEnd4 = sqrt(l2^2 - (xEnd4 - l1).^2); 
xEnd = linspace(-l1-l2,l1+l2,100);
yEnd = sqrt((l1 + l2)^2 - xEnd.^2);                 % 红色圆弧(外侧

xEnd1fill= linspace(-28,32,100);
yEnd1fill = sqrt((l1 + l2 - 5 )^2 - xEnd1fill.^2);    % 绿色圆弧（外侧
xEnd2fill = linspace(l1-l2-5,-4.51,100);       
yEnd2fill = sqrt((l1 - l2-5)^2 - xEnd2fill.^2);       % 绿色圆弧 (右侧
xEnd4fill = linspace(-4.51, 32, 100);                 % 黑色圆弧 (右侧
yEnd4fill = sqrt(l2^2 - (xEnd4fill - l1).^2); 
xFillVerLeft =[-28 -28];                              % x=-28
yFillVerLeft =[0 sqrt((l1 + l2 - 5 )^2 - 28^2)];      
xFillVerRight = [32 32];                              % x= 32
yFillVerRight = [sqrt((l1 + l2 - 5 )^2 - 32^2) sqrt(l2^2 - (32 - l1).^2) ];

fill([xFillVerLeft,xEnd1fill,fliplr(xEnd4fill),fliplr(xEnd2fill)] ,[-yFillVerLeft,-yEnd1fill,-fliplr(yEnd4fill),-fliplr(yEnd2fill)], [0.9, 0.9, 0.9], 'EdgeColor', 'none'); % 填充上半圆区域为浅灰色

plot(xEnd1, -yEnd1 , 'g--','LineWidth',1);             % Safe workspace
plot(xEnd2, -yEnd2 , 'g--','LineWidth',1);             % Safe workspace
plot(xEnd3, -yEnd3 , 'r--','LineWidth', 1);            % 红色圆弧（内
plot(xEnd, -yEnd , 'r--','LineWidth',1);               % 红色/黑色圆弧（外侧    
plot(xEnd4,-yEnd4,'k--');    % 黑色圆弧 (右侧
plot([-l1-l2, l1-l2-5], [0, 0], 'k--');               % 工作空间的水平边界
plot(xFillVerLeft,-yFillVerLeft,'k--',xFillVerRight,-yFillVerRight,'k--')
plot(0, 0, '.', 'MarkerSize', 20, 'Color', 'k');    % plot base point

q = [0,2];
x1 = l1 * cos(q(1));                   % 第一个关节位置
y1 = l1 * sin(q(1));
x2 = x1 + l2 * cos(q(1) + q(2));       % 末端执行器位置
y2= y1 + l2 * sin(q(1) + q(2));
plot([x1, x2], [-y1, -y2], 'b', 'LineWidth', 2);      % 绘制第二段机械臂
plot([0, x1], [0, -y1], 'r', 'LineWidth', 2);        % 绘制第一段机械臂

% arrow
quiver([-l1-l2 l1-l2],[0 0],[5 -5],[0 0],'AutoScale', 'off','LineWidth',2,'LineStyle',':','Color',[0.8, 0.6, 0.2]);

% 2D_XYF,在minF
scatter(store_minXYF(1,:),-store_minXYF(2,:),20,store_minXYF(3,:),'filled');
cb=colorbar;
cb.Label.String = 'F (N)';

% 标注工作空间
text(-20, -55, 'Safe Workspace','FontSize', 12, 'FontWeight', 'bold');
text(-60,-72,'x = -28cm,fully closed','FontSize',12,'FontWeight', 'bold')
text(20,-72,'x = 32cm,fully open','FontSize',12,'FontWeight', 'bold')
text(-70,5,'5cm','FontWeight', 'bold','FontSize',12)
text(-15,5,'5cm','FontWeight', 'bold','FontSize',12)
text(0,6,'Base Point','FontSize',12,'FontWeight', 'bold')

set(gca,'FontSize',14)
axis([-l1-l2-10 l1+l2+10 -90 30]);
xlabel('x(cm)')
ylabel('y(cm)')
saveas(gcf, 'valid/case1_25', 'epsc');  % save
%% Case1:2D_QminF,并有JointSpace
% numSample = 200;        % 画JointSpace
% store_q1Mat = zeros(numSample,numSample);
% store_q2Mat = zeros(numSample,numSample);
% index1 = 1;
% for x = linspace(d_close,d_open,numSample)
%     index2 = 1;
%     for y = linspace(get_yBoundary1(l1,l2,x),get_yBoundary2(l1,l2,x),numSample)
%         q = inv_kine (l1, l2, x, y);                                    % IK
%         store_q1Mat(index1,index2)=q(1);
%         store_q2Mat(index1,index2)=q(2);
%         index2 = index2 + 1;                                            % update Index
%     end
%     index1 = index1 + 1;
% end
% 
% figure
% grid on
% hold on
% scatter(store_q1Mat,store_q2Mat,[], [0.9, 0.9, 0.9], 'filled')  % 灰色的joint Space
% xlabel('q_1 (radian)')
% ylabel('q_2 (radian)')
% axis([-0.5 2.5 0 3]);
% 
% scatter(store_minQ1,store_minQ2,20,store_minXYF(3,:),'filled'); % QminF
% cb=colorbar;
% cb.Label.String = 'F (N)';
% 
% set(gca,'FontSize',14)
% xlabel('q1')
% ylabel('q2')

%% case1 3D_XYF_1,标注minimum
% store_xMat = store1_x';
% store_yMat = store1_y;
% store_FMat = store1_y';
% store_q1Mat = store1_q1';
% store_q2Mat = store1_q2';
% 
% x = reshape(store_xMat', 1, []); % 提取 x 
% y = reshape(store_yMat', 1, []); % 提取 y 
% v = reshape(store_FMat', 1, []); % 提取 F 
% [xq, yq] = meshgrid(d_close:.5:d_open, 0:.5:l1+l2); % 设定范围
% vq = griddata(x, y, v, xq, yq); 
% figure
% mesh(xq, yq, vq) % 绘制网格线图
% hold on
% colorbar;
% h1=plot3(store_minXYF(1,:),store_minXYF(2,:),store_minXYF(3,:),'r','LineWidth',2);
% legend(h1, 'Minimum Force', 'Location', 'best');
% 
% xlabel('x(cm)')
% ylabel('y(cm)')
% zlabel('F(N)')
%% 展示采样
% figure
% hold on
% scatter(store1_x,store1_y);
%% 动态  IK test 
 
% for index6 = 60
%     
%     q1 = (store1_q1(:,index6))';
%     q2 = (store1_q2(:,index6))';
%     q = [q1;q2];
%     
%     x1 = zeros(1, length(q1));  % 初始化末端执行器位置
%     y1 = zeros(1, length(q2));
%     x2 = zeros(1, length(q1));
%     y2 = zeros(1, length(q2));
%     
%     for i = 1:length(q1)
%         
%         x1(i) = l1 * cos(q(1, i));                         % 第一个关节位置
%         y1(i) = l1 * sin(q(1, i));
%         
%         
%         x2(i) = x1(i) + l2 * cos(q(1, i) + q(2, i));       % 末端执行器位置
%         y2(i) = y1(i) + l2 * sin(q(1, i) + q(2, i));
%     end
%   
%     
%     figure;
%     hold on;
%     xlabel('X Position');
%     ylabel('Y Position');
%     title('Two-Link Arm Motion with Pose');
%     grid on;
%     axis equal;
%     
%     for i = 1:length(q1)                % 动态显示机械臂的姿态
%     
%         cla;                              % 清除上一次的绘图
%         plot([0, x1(i)], [0, y1(i)], 'r', 'LineWidth', 2);         % 绘制第一段机械臂
%         plot([x1(i), x2(i)], [y1(i), y2(i)], 'b', 'LineWidth', 2); % 绘制第二段机械臂
%         plot(x2(1:i), y2(1:i), 'k--');                             % 绘制末端执行器轨迹
%         axis([-l1-l2 l1+l2 -l1-l2 l1+l2]);
%         pause(0.01);                                                % 暂停，使动画可视化
%     end
%     
%    
% end
% 
% hold off;

