function plot_jointSpace_1(matrix_q1,matrix_q2)
% for plotting, the shape of matrix is changed


% joint Space
figure
grid on
hold on
scatter(matrix_q1,matrix_q2,[], [0.9, 0.9, 0.9], 'filled') 

% Set axis and labels
xlabel('q_1 (radian)')
ylabel('q_2 (radian)')
axis([-2 1 -3.5 -0.5]);
set(gca,'FontSize',14);

end

