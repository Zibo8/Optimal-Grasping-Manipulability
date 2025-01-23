function y = get_y_ub(x)
    
    % The function aims to calculate value of y_ub(x)
    % Input:  x \in [-28 32]
    % Output: y_ub(x) - upper boundary of y

    % Length of robot arm_1, the unit is cm
    l1 = 32.3097;       
    % Length of robot arm_2, the unit is cm
    l2 = 38.1395;

    % use 5 cm to avoid singularity of robot
    % l1-l2-5 = -10.8298
    if x<(l1-l2-5)
       y = 0;
    elseif x>=(l1-l2-5) && x< -4.5095
        y = -sqrt( (abs( -l1 + l2) + 5)^2 - x^2);
    else
         y = -sqrt(l2^2 - (x-l1)^2);
    end

end
