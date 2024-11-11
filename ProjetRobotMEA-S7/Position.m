function [q_dot] = Position(epsilon_P, P_d_dot, K_P, J, W_e)
    % Position - Calcule la vitesse articulaire q_dot pour atteindre la position souhaitée
    % epsilon_P: Erreur de position entre la position souhaitée et la position actuelle
    % P_d_dot: Vitesse désirée dans l'espace cartésien (vecteur 3D)
    % K_P: Gain proportionnel pour le contrôle de position
    % J: Jacobienne calculée au point actuel

    % Calculez la vitesse de position souhaitée dans l'espace cartésien (linéaire et angulaire)
    P_dot_e_linear = P_d_dot + K_P .* epsilon_P; % Vitesse linéaire désirée

    % Combinez dans un vecteur 6x1 les vitesses linéaires et angulaires instantanées pour P_dot_e
    P_dot_e = [P_dot_e_linear; W_e];

    % Transformez la vitesse de position souhaitée en vitesses articulaires en utilisant la pseudo-inverse de J
    J_pseudo_inv = pinv(J); % Calcule la pseudo-inverse de la Jacobienne
    q_dot = J_pseudo_inv * P_dot_e; % Vitesse articulaire nécessaire pour corriger la position
end
