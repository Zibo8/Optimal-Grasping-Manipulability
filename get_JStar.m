function J_star = get_JStar(l1, l2, q1, q2)
    % Calculate the inverse of transform Jacobian matrix for a 2-link manipulator
    % Input: l1 - length of link 1
    %        l2 - length of link 2
    %        q1 - joint angle 1 in radians
    %        q2 - joint angle 2 in radians
    % Output:Inverse of Transform Jacobian matrix
    % unit:  radian for q_1 and q_2
    
    J11 = -l1 * sin(q1) - l2 * sin(q1 + q2);
    J12 = -l2 * sin(q1 + q2);
    J21 = l1 * cos(q1) + l2 * cos(q1 + q2);
    J22 = l2 * cos(q1 + q2);
    
    % Calculate jocabian matirx
    Jmatrix = [J11, J12;
               J21, J22];

    % Calculate transform matirx
    J_trans = Jmatrix';
    
    % Calculate the inverse of transform Jacobian matrix
    J_star = pinv(J_trans);
    
end
