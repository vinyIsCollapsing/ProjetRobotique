function [joint_positions, joint_orientations, P_e, A_e] = MGD(alpha, r, theta, d)
    % MGD - Calcula a posição e a orientação de cada junta do robô e
    % retorna P_e e A_e
    % joint_positions: Matriz contendo as posições de cada junta
    % joint_orientations: Matriz contendo as orientações de cada junta
    % P_e: Vetor 3x1 contendo a posição do efetuador final (end-effector)
    % A_e: Matriz 3x3 de rotação do efetuador final (end-effector)
    
    T_base_to_joint = eye(4);
    joint_positions = zeros(3, length(alpha) + 1); 
    joint_orientations = zeros(3, 3, length(alpha) +1);
    
    % Store the initial position (base position)T and initial orientation
    joint_positions(:, 1) = [0; 0; 0];
    joint_orientations(:,:,1) = [1 0 0; 0 1 0; 0 0 1];

    % Loop to calculate transformations and store joint positions
    for i = 1:length(alpha)
        T_i = dh_modified(alpha(i), r(i), theta(i), d(i)); % Transformation for each joint
        T_base_to_joint = T_base_to_joint * T_i; % Accumulated transformation
        joint_positions(:, i + 1) = T_base_to_joint(1:3, 4); % Extract position of joint
        joint_orientations(:,:,i+1) = T_base_to_joint(1:3, 1:3); % Extract rotation of joint
    end

    % Extract P_e and A_e 
    P_e = T_base_to_joint(1:3, 4); % Final position of the end-effector
    A_e = T_base_to_joint(1:3,1:3); % Final orientation of the end-effector

    % Plot the robot
    % figure;
    % hold on;
    % grid on;
    % axis([-750 750 -750 750 0 800]);
    % xlabel('X'); ylabel('Y'); zlabel('Z');
    % title('Robot Structure and Joint Axes');

    % Draw the skeleton (lines connecting each joint)
    % for i = 1:length(alpha)
    %     plot3(joint_positions(1, i:i+1), joint_positions(2, i:i+1), joint_positions(3, i:i+1), 'ko-', 'LineWidth', 2);
    % end

    % Draw the coordinate frames for each joint
    % for i = 1:length(alpha) + 1
    %     plot_coordinate_frame(joint_positions(:, i), T_base_to_joint(1:3, 1:3), 50);
    % end

    % hold off;
end

% Function to calculate the transformation matrix using modified DH parameters
function T = dh_modified(alpha, r, theta, d)
    T = [cos(theta), -sin(theta), 0, d;
         cos(alpha)*sin(theta), cos(alpha)*cos(theta), -sin(alpha), -r*sin(alpha);
         sin(alpha)*sin(theta), sin(alpha)*cos(theta), cos(alpha), r*cos(alpha);
         0, 0, 0, 1];
end

% Function to draw coordinate frame for a given position and orientation
% function plot_coordinate_frame(position, rotation_matrix, axis_length)
%     x_axis = rotation_matrix(:, 1) * axis_length;
%     y_axis = rotation_matrix(:, 2) * axis_length;
%     z_axis = rotation_matrix(:, 3) * axis_length;
%     
%     % Plot X, Y, Z axes (red, green, blue)
%     plot3([position(1), position(1) + x_axis(1)], [position(2), position(2) + x_axis(2)], [position(3), position(3) + x_axis(3)], 'r', 'LineWidth', 1.5);
%     plot3([position(1), position(1) + y_axis(1)], [position(2), position(2) + y_axis(2)], [position(3), position(3) + y_axis(3)], 'g', 'LineWidth', 1.5);
%     plot3([position(1), position(1) + z_axis(1)], [position(2), position(2) + z_axis(2)], [position(3), position(3) + z_axis(3)], 'b', 'LineWidth', 1.5);
% end