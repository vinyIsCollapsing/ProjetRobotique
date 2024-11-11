function J = Jacobiana(theta, alpha, r, d)
    num_juntas = length(theta);
    T = eye(4);

    Jv = zeros(3, num_juntas);  % Parte linear da Jacobiana
    Jw = zeros(3, num_juntas);  % Parte angular da Jacobiana

    % Posição e orientação iniciais
    z = [0; 0; 1];  % Eixo Z inicial
    p = [0; 0; 0];  % Origem inicial

    for i = 1:num_juntas
        % Calcule a transformação da junta atual
        T_i = dh_modified(alpha(i), r(i), theta(i), d(i));
        T = T * T_i;

        % Posição e eixo da junta atual
        p_i = T(1:3, 4); % Posição da ponta da junta i
        z_i = T(1:3, 3); % Eixo Z da junta i

        % Calcular Jv e Jw para a junta atual
        Jv(:, i) = cross(z, (p_i - p)); % Velocidade linear
        Jw(:, i) = z;                   % Velocidade angular

        % Atualizar p e z para a próxima iteração
        z = z_i;
        p = p_i;
    end

    % Montar a Jacobiana final combinando Jv e Jw
    J = [Jv; Jw];
end

function T = dh_modified(alpha, r, theta, d)
    % Função de matriz de transformação usando parâmetros DH modificados
    T = [cos(theta), -sin(theta), 0, r;
         cos(alpha)*sin(theta), cos(alpha)*cos(theta), -sin(alpha), -d*sin(alpha);
         sin(alpha)*sin(theta), sin(alpha)*cos(theta), cos(alpha), d*cos(alpha);
         0, 0, 0, 1];
end
