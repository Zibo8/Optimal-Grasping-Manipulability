function q = inv_kinematics(l1, l2, x, y)
    % inverse kinematics
    % Calculate possible solutions for q1 and q2
    % Input: l1 - length of link 1 in cm
    %        l2 - length of link 2 in cm
    %        x - coordinate in cm
    %        y - coordinate in cm
    % Output: q_matrix is 1x2 matrix, include q_1 and q_2 in radians
   

    % Distance from origin to point (x, y)
    r = sqrt(x^2 + y^2);  
    
    % Check if the point is reachable
    if r > (l1 + l2) || r < abs(l1 - l2)
        error('Point (%f, %f) is out of reach for the robot.', x, y);
    end

    % Law of Cosines for angles
    cos_q2 = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2);
    sin_q2_up = sqrt(1 - cos_q2^2);     % Elbow-Up
    sin_q2_down = -sqrt(1 - cos_q2^2);  % Elbow-Down
    
    q2_up = atan2(sin_q2_up, cos_q2);       % Elbow-Up solution
    q2_down = atan2(sin_q2_down, cos_q2);   % Elbow-Down solution
    
    % Angle q1 depends on q2
    k1 = l1 + l2 * cos_q2;
    k2_up = l2 * sin_q2_up;         % Elbow-Up
    k2_down = l2 * sin_q2_down;     % Elbow-Down
    
    q1_up = atan2(y, x) - atan2(k2_up, k1);      % Elbow-Up solution
    q1_down = atan2(y, x) - atan2(k2_down, k1);  % Elbow-Down solution
    
    % Choose solution (Toggle between 'up' and 'down')
    solutionType = 'down'; % Change this to 'up' or 'down' as needed
    
    if strcmp(solutionType, 'up')
        q = [q1_up, q2_up];
    elseif strcmp(solutionType, 'down')
        q = [q1_down, q2_down];
    else
        error('Invalid solution type. Choose "up" or "down".');
    end
end
