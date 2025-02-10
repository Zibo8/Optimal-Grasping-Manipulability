function [l1,l2,d_close,d_open,torque,vector_x,vector_y,number_x,number_y] = initialize_2(intervals)

l1 = 32.3097;       % length of robot arm_1,    the unit is cm
l2 = 38.1395;       % length of robot arm_2,    the unit is cm

d_close = -28;      % robot arm fully closed,   the unit is cm            
d_open  =  32;      % robot arm fully open,     the unit is cm  

torque =  -10000;   % torque of Joint,          the unit is N*cm
                    % clockwise direction is negative

% sampling intervals = 0.1 cm
% x is samplede in [-28 32]
% y is samplede in [-65.4492 0], y in [-65.4492 0], as -l1 - l2 + 5 = -65.4492 cm
vector_x = -28:intervals:32;            
vector_y = -(0:intervals:57)';

% determin numer_x and number_y based on vector_x and vector_y
% 10mm intervals-> 601, 5mm intervals-> 1201, 25mm intervals-> 241
number_x = length(vector_x); 
% 10mm intervals-> 655, 5mm intervals-> 1309, 25mm intervals-> 262
number_y = length(vector_y);  
end

