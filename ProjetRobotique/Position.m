function [q_dot] = Position(epsilon_P, P_d_dot, K_P, J)
    % P_d: Posicao desejada no espaco cartesiano
    % P_e: Posicao atual no espaco cartesiano
    % P_d_dot: Velocidade desejada no espaco cartesiano (vetor 3D)
    % K_P: Ganho proporcional de posição
    % J: Jacobiana calculada no ponto atual

    % Calcule a velocidade de posição desejada no espaço cartesiano (linear e angular)
    P_dot_e_linear = P_d_dot + K_P .* epsilon_P; % Velocidade linear desejada

    P_dot_e_angular = [0; 0; 0]; % Velocidade angular desejada (defina conforme necessário)
    
    % Combine em um vetor 6x1 para P_dot_e
    P_dot_e = [P_dot_e_linear; P_dot_e_angular];

    % Transforme a velocidade de posição desejada para as velocidades articulares usando a pseudoinversa de J
    J_pseudo_inv = pinv(J); % Calcula a pseudoinversa da Jacobiana
    q_dot = J_pseudo_inv * P_dot_e; % Velocidade articular necessária para corrigir a posição
end