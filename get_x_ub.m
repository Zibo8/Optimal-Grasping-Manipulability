function x = get_x_ub(y)

    % The function aims to calculate value of x_ub(x)
    % Input:  y \in [-57 0]
    % Output: x_ub(x) - lower boundary of x
    % Unit: cm

    % Length of robot arm_1, the unit is cm
    l1 = 32.3097;       
    % Length of robot arm_2, the unit is cm
    l2 = 38.1395;

     % Use 5cm, to avoid singularity
    if y>-9.847
       x = - sqrt( (abs( -l1 + l2) + 5)^2 - y^2);
    elseif y<=-9.847 && y> -38
       x = l1 - sqrt(l2^2-y^2);
    else
         x = 32;
    end

end