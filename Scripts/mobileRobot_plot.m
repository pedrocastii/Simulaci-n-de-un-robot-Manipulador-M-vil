%% MOBILE ROBOT REPRESENTATION

% Mobile robot's params
params=[L,W,r.w];
%           L = Mobile robot footprint (rectangle) length
%           W = Mobile robot footprint (rectangle) width
%           r = Wheel radius of mobile robot
%           w = Wheel width of mobile robot


    % Mobile robot base
    m_x = [states(1)-(params(1)/2), states(1)-(params(1)/2), states(1)+(params(1)/2), states(1)+(params(1)/2)]; % Base vertices (X-coordinates)
    m_y = [states(2)+(params(2)/2), states(2)-(params(2)/2), states(2)-(params(2)/2), states(2)+(params(2)/2)]; % Base vertices (Y-coordinates)
    R = [cos(states(3)) -sin(states(3));sin(states(3)) cos(states(3))]; % Rotation matrix
    m_v = R*[m_x(:)-mean(m_x) m_y(:)-mean(m_y)]'; % Center and rotate
    m_x = m_v(1,:)+mean(m_x); % Restore original position
    m_y = m_v(2,:)+mean(m_y); % Restore original position
    
    % Mobile robot right wheel
    w_r_c_x = states(1)+(params(2)/2)*sin(states(3)); % Wheel center X-coordinate
    w_r_c_y = states(2)-(params(2)/2)*cos(states(3)); % Wheel center Y-coordinate
    w_r_x = [w_r_c_x-params(3), w_r_c_x-params(3), w_r_c_x+params(3), w_r_c_x+params(3)]; % Wheel vertices (X-coordinates)
    w_r_y = [w_r_c_y+(params(4)/2), w_r_c_y-(params(4)/2), w_r_c_y-(params(4)/2), w_r_c_y+(params(4)/2)]; % Wheel vertices (Y-coordinates)
    R = [cos(states(3)) -sin(states(3));sin(states(3)) cos(states(3))]; % Rotation matrix
    w_r_v = R*[w_r_x(:)-mean(w_r_x) w_r_y(:)-mean(w_r_y)]'; % Center and rotate
    w_r_x = w_r_v(1,:)+mean(w_r_x); % Restore original position
    w_r_y = w_r_v(2,:)+mean(w_r_y); % Restore original position
    
    % Mobile robot left wheel
    w_l_c_x = states(1)-(params(2)/2)*sin(states(3)); % Wheel center X-coordinate
    w_l_c_y = states(2)+(params(2)/2)*cos(states(3)); % Wheel center Y-coordinate
    w_l_x = [w_l_c_x-params(3), w_l_c_x-params(3), w_l_c_x+params(3), w_l_c_x+params(3)]; % Wheel vertices (X-coordinates)
    w_l_y = [w_l_c_y+(params(4)/2), w_l_c_y-(params(4)/2), w_l_c_y-(params(4)/2), w_l_c_y+(params(4)/2)]; % Wheel vertices (Y-coordinates)
    R = [cos(states(3)) -sin(states(3));sin(states(3)) cos(states(3))]; % Rotation matrix
    w_l_v = R*[w_l_x(:)-mean(w_l_x) w_l_y(:)-mean(w_l_y)]'; % Center and rotate
    w_l_x = w_l_v(1,:)+mean(w_l_x); % Restore original position
    w_l_y = w_l_v(2,:)+mean(w_l_y); % Restore original position

 