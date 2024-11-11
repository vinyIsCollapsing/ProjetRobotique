function [joint_positions, joint_orientations, P_e, A_e] = MGD(alpha, r, theta, d, iteration)
    % MGD - Calcule la position et l'orientation de chaque articulation du robot et
    % retourne P_e et A_e
    % joint_positions: Matrice contenant les positions de chaque articulation
    % joint_orientations: Matrice contenant les orientations de chaque articulation
    % P_e: Vecteur 3x1 contenant la position du bout de l'effecteur (end-effector)
    % A_e: Matrice de rotation 3x3 de l'effecteur final (end-effector)
    
    T_base_to_joint = eye(4);
    joint_positions = zeros(3, length(alpha) + 1); 
    joint_orientations = zeros(3, 3, length(alpha) + 1);
    
    % Stocker la position initiale (position de la base) et l'orientation initiale
    joint_positions(:, 1) = [0; 0; 0];
    joint_orientations(:,:,1) = [1 0 0; 0 1 0; 0 0 1];

    % Boucle pour calculer les transformations et stocker les positions des articulations
    for i = 1:length(alpha)
        T_i = dh_modified(alpha(i), r(i), theta(i), d(i)); % Transformation pour chaque articulation
        T_base_to_joint = T_base_to_joint * T_i; % Transformation accumulée
        joint_positions(:, i + 1) = T_base_to_joint(1:3, 4); % Extraire la position de l'articulation
        joint_orientations(:,:,i+1) = T_base_to_joint(1:3, 1:3); % Extraire la rotation de l'articulation
    end

    % Extraire P_e et A_e 
    P_e = T_base_to_joint(1:3, 4); % Position finale de l'effecteur
    A_e = T_base_to_joint(1:3,1:3); % Orientation finale de l'effecteur
    
    if iteration == 0
        % Modélisation
        % Tracer le robot
        figure;
        hold on;
        grid on;
        axis([-750 750 -750 750 0 800]);
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title('Structure du robot et axes des articulations');
        
        % Dessiner le squelette (lignes reliant chaque articulation)
        for i = 1:length(alpha)
             plot3(joint_positions(1, i:i+1), joint_positions(2, i:i+1), joint_positions(3, i:i+1), 'ko-', 'LineWidth', 2);
        end
        
        % Dessiner les cadres de coordonnées pour chaque articulation
        for i = 1:length(alpha) + 1
             plot_coordinate_frame(joint_positions(:, i), T_base_to_joint(1:3, 1:3), 50);
        end
        
        hold off;
    end

end

% Fonction pour calculer la matrice de transformation utilisant les paramètres DH modifiés
function T = dh_modified(alpha, r, theta, d)
    T = [cos(theta), -sin(theta), 0, d;
         cos(alpha)*sin(theta), cos(alpha)*cos(theta), -sin(alpha), -r*sin(alpha);
         sin(alpha)*sin(theta), sin(alpha)*cos(theta), cos(alpha), r*cos(alpha);
         0, 0, 0, 1];
end

% Fonction pour dessiner le cadre de coordonnées pour une position et une orientation données
function plot_coordinate_frame(position, rotation_matrix, axis_length)
    x_axis = rotation_matrix(:, 1) * axis_length;
    y_axis = rotation_matrix(:, 2) * axis_length;
    z_axis = rotation_matrix(:, 3) * axis_length;
    
    % Tracer les axes X, Y, Z (rouge, vert, bleu)
    plot3([position(1), position(1) + x_axis(1)], [position(2), position(2) + x_axis(2)], [position(3), position(3) + x_axis(3)], 'r', 'LineWidth', 1.5);
    plot3([position(1), position(1) + y_axis(1)], [position(2), position(2) + y_axis(2)], [position(3), position(3) + y_axis(3)], 'g', 'LineWidth', 1.5);
    plot3([position(1), position(1) + z_axis(1)], [position(2), position(2) + z_axis(2)], [position(3), position(3) + z_axis(3)], 'b', 'LineWidth', 1.5);
end
