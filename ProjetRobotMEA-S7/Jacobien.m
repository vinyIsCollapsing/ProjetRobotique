function J = Jacobien(theta, alpha, r, d)
    num_juntas = length(theta);
    T = eye(4);

    Jv = zeros(3, num_juntas);  % Partie linéaire de la Jacobienne
    Jw = zeros(3, num_juntas);  % Partie angulaire de la Jacobienne

    % Position et orientation initiales
    z = [0; 0; 1];  % Axe Z initial
    p = [0; 0; 0];  % Origine initiale

    for i = 1:num_juntas
        % Calculer la transformation de l'articulation actuelle
        T_i = dh_modified(alpha(i), r(i), theta(i), d(i));
        T = T * T_i;

        % Position et axe de l'articulation actuelle
        p_i = T(1:3, 4); % Position de l'extrémité de l'articulation i
        z_i = T(1:3, 3); % Axe Z de l'articulation i

        % Calculer Jv et Jw pour l'articulation actuelle
        Jv(:, i) = cross(z, (p_i - p)); % Vitesse linéaire
        Jw(:, i) = z;                   % Vitesse angulaire

        % Mettre à jour p et z pour l'itération suivante
        z = z_i;
        p = p_i;
    end

    % Construire la Jacobienne finale en combinant Jv et Jw
    J = [Jv; Jw];
end

function T = dh_modified(alpha, r, theta, d)
    % Fonction de matrice de transformation utilisant les paramètres DH modifiés
    T = [cos(theta), -sin(theta), 0, r;
         cos(alpha)*sin(theta), cos(alpha)*cos(theta), -sin(alpha), -d*sin(alpha);
         sin(alpha)*sin(theta), sin(alpha)*cos(theta), cos(alpha), d*cos(alpha);
         0, 0, 0, 1];
end
