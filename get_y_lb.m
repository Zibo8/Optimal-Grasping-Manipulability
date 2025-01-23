function y = get_y_lb(x)
    
    % The function aims to calculate value of y_lb(x)
    % Input:  x \in [-28 32]
    % Output: y_lb(x) - lower boundary of y
    % Unit: cm

    % Length of robot arm_1, the unit is cm
    l1 = 32.3097;       
    % Length of robot arm_2, the unit is cm
    l2 = 38.1395;

    % Use 5cm, to avoid external singularity
    y = -sqrt((l1 + l2 - 5 )^2 - x.^2);

end