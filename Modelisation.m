% Modelisation
% Function to calculate the transformation matrix using modified DH parameters
function T = dh_modified(alpha, r, theta, d)
    T = [cos(theta), -sin(theta), 0, d;
         cos(alpha)*sin(theta), cos(alpha)*cos(theta), -sin(alpha), -r*sin(alpha);
         sin(alpha)*sin(theta), sin(alpha)*cos(theta), cos(alpha), r*cos(alpha);
         0, 0, 0, 1];
end

% Define DH parameters for the Robotis-H (as before)
alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
r = [159, 0, 0, 258, 0, 0];
theta = [0; -1.4576453; -0.898549163; 0; 0; -pi];
d = [0, 0, 265.69, 30, 0, 123];

% Initialize the transformation matrix from base to each joint
T_base_to_joint = eye(4);
joint_positions = zeros(3, length(alpha) + 1); % Store positions of each joint

% Store the initial position (base position)
joint_positions(:, 1) = [0; 0; 0];

% Loop to calculate transformations and store joint positions
for i = 1:length(alpha)
    T_i = dh_modified(alpha(i), r(i), theta(i), d(i)); % Transformation for each joint
    T_base_to_joint = T_base_to_joint * T_i; % Accumulated transformation
    joint_positions(:, i + 1) = T_base_to_joint(1:3, 4); % Extract position of joint
end

% Dessin du robot
% Plot the robot
figure;
hold on;
grid on;
axis ([-750 750 -750 750 0 800]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Robot Structure and Joint Axes');

% Draw the skeleton (lines connecting each joint)
for i = 1:length(alpha)
    plot3(joint_positions(1, i:i+1), joint_positions(2, i:i+1), joint_positions(3, i:i+1), 'ko-', 'LineWidth', 2);
end

% Draw the coordinate frames for each joint
for i = 1:length(alpha) + 1
    plot_coordinate_frame(joint_positions(:, i), T_base_to_joint(1:3, 1:3), 50);
end

hold off;

% Function to draw coordinate frame for a given position and orientation
function plot_coordinate_frame(position, rotation_matrix, axis_length)
    x_axis = rotation_matrix(:, 1) * axis_length;
    y_axis = rotation_matrix(:, 2) * axis_length;
    z_axis = rotation_matrix(:, 3) * axis_length;
    
    % Plot X, Y, Z axes (red, green, blue)
    plot3([position(1), position(1) + x_axis(1)], [position(2), position(2) + x_axis(2)], [position(3), position(3) + x_axis(3)], 'r', 'LineWidth', 1.5);
    plot3([position(1), position(1) + y_axis(1)], [position(2), position(2) + y_axis(2)], [position(3), position(3) + y_axis(3)], 'g', 'LineWidth', 1.5);
    plot3([position(1), position(1) + z_axis(1)], [position(2), position(2) + z_axis(2)], [position(3), position(3) + z_axis(3)], 'b', 'LineWidth', 1.5);
end

% Commande articulaire
% Loi Trapeze
function Param = CalculeTrapeze(robot, qi, qf, duree)
    % Constantes de velocidade e aceleração máxima
    q_dot_max = robot.q_dot_max;
    q_ddot_max = robot.q_ddot_max;

    % Distância a ser percorrida
    delta_q = abs(qf - qi);
    
    % Calcular o tempo necessário para a fase de aceleração (ou desaceleração)
    t1 = q_dot_max / q_ddot_max;
    d1 = 0.5 * q_ddot_max * t1^2; % Distância percorrida durante a aceleração

    if delta_q <= 2 * d1
        % Caso a distância seja pequena demais para atingir a velocidade máxima
        t1 = sqrt(delta_q / q_ddot_max);
        tf = 2 * t1;
        d1 = delta_q / 2;
    else
        % Caso normal, com fase de velocidade constante
        d2 = delta_q - 2 * d1; % Distância percorrida na fase de velocidade constante
        t2 = d2 / q_dot_max;   % Tempo da fase de velocidade constante
        tf = 2 * t1 + t2;
    end

    % Ajustar tf se um tempo total específico duree for fornecido
    if duree > 0 && duree > 2 * t1
        tf = duree;
        t2 = tf - 2 * t1;
    end

    % Armazenar os parâmetros em uma estrutura
    Param.qi = qi;
    Param.qf = qf;
    Param.t1 = t1;
    Param.tf = tf;
    Param.q_dot_max = q_dot_max;
    Param.q_ddot_max = q_ddot_max;
end

function [q, q_dot, q_ddot] = CalculeQ(robot, Param, t)
    % Desestruturação dos parâmetros
    qi = Param.qi;
    qf = Param.qf;
    t1 = Param.t1;
    tf = Param.tf;
    q_dot_max = Param.q_dot_max;
    q_ddot_max = Param.q_ddot_max;

    delta_q = qf - qi;  % Distância total a ser percorrida
    sign_delta_q = sign(delta_q);  % Direção do movimento

    % Inicialização dos resultados
    q = qi;
    q_dot = 0;
    q_ddot = 0;

    % Verificar em qual fase o tempo t está e calcular posição, velocidade e aceleração
    if t <= t1
        % Fase de aceleração
        q = qi + 0.5 * q_ddot_max * t^2 * sign_delta_q;
        q_dot = q_ddot_max * t * sign_delta_q;
        q_ddot = q_ddot_max * sign_delta_q;
    elseif t <= (tf - t1)
        % Fase de velocidade constante
        q = qi + (0.5 * q_ddot_max * t1^2 + q_dot_max * (t - t1)) * sign_delta_q;
        q_dot = q_dot_max * sign_delta_q;
        q_ddot = 0;
    elseif t <= tf
        % Fase de desaceleração
        t_dec = t - (tf - t1);
        q = qf - 0.5 * q_ddot_max * t_dec^2 * sign_delta_q;
        q_dot = q_ddot_max * (tf - t) * sign_delta_q;
        q_ddot = -q_ddot_max * sign_delta_q;
    else
        % Após tf, posição é exatamente qf
        q = qf;
        q_dot = 0;
        q_ddot = 0;
    end

    % Garantir que a posição não ultrapasse qf em nenhuma fase
    if sign_delta_q > 0
        q = min(q, qf);  % Movimento positivo, garantir que q não ultrapasse qf
    else
        q = max(q, qf);  % Movimento negativo, garantir que q não fique abaixo de qf
    end
end

% Definir limites do robô como uma estrutura
robot = struct();
robot.q_dot_max = 3.2;  % Velocidade máxima (rad/s)
robot.q_ddot_max = 30;  % Aceleração máxima (rad/s^2)

% Parâmetros iniciais
qi = -pi/2;          % Posição inicial (rad)
qf = pi/2;       % Posição final (rad)
duree = 0;       % Tempo total desejado (s)

% Calcular parâmetros da trajetória trapézio
Param = CalculeTrapeze(robot, qi, qf, duree);

% Cálculo da trajetória ao longo do tempo
t_values = linspace(0, Param.tf, 100);  % Vetor de tempo de 0 até tf com 100 pontos
q_values = zeros(size(t_values));       % Vetor para armazenar posições da junta
q_dot_values = zeros(size(t_values));   % Vetor para armazenar velocidades
q_ddot_values = zeros(size(t_values));  % Vetor para armazenar acelerações

for i = 1:length(t_values)
    [q_values(i), q_dot_values(i), q_ddot_values(i)] = CalculeQ(robot, Param, t_values(i));
end

% Plotar a posição da junta ao longo do tempo
figure;
subplot(3, 1, 1);
plot(t_values, q_values, 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Posição (rad)');
title('Posição da Junta');
grid on;

% Plotar a velocidade da junta ao longo do tempo
subplot(3, 1, 2);
plot(t_values, q_dot_values, 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Velocidade (rad/s)');
title('Velocidade da Junta');
grid on;

% Plotar a aceleração da junta ao longo do tempo
subplot(3, 1, 3);
plot(t_values, q_ddot_values, 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Aceleração (rad/s²)');
title('Aceleração da Junta');
grid on;

% Commande articulaire
% Loi Polynomiale
function [q, q_dot, q_ddot] = CalculePolinomio(qi, qf, tf, t)
    % Calcular os coeficientes da função cúbica para trajetória polinomial
    a0 = qi;
    a1 = 0;
    a2 = (3 * (qf - qi)) / tf^2;
    a3 = (-2 * (qf - qi)) / tf^3;

    % Calcular posição, velocidade e aceleração em t
    q = a0 + a1 * t + a2 * t^2 + a3 * t^3;
    q_dot = a1 + 2 * a2 * t + 3 * a3 * t^2;
    q_ddot = 2 * a2 + 6 * a3 * t;
end

% Parâmetros iniciais
qi = -pi;          % Posição inicial (rad)
qf = pi;       % Posição final (rad)
tf = 2;          % Tempo total desejado (s)

% Cálculo da trajetória ao longo do tempo
t_values = linspace(0, tf, 100);
q_values = zeros(size(t_values)); 
q_dot_values = zeros(size(t_values)); 
q_ddot_values = zeros(size(t_values)); 

for i = 1:length(t_values)
    [q_values(i), q_dot_values(i), q_ddot_values(i)] = CalculePolinomio(qi, qf, tf, t_values(i));
end

% Plotar a posição, velocidade e aceleração ao longo do tempo
figure;

subplot(3, 1, 1);
plot(t_values, q_values, 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Posição (rad)');
title('Posição da Junta');
grid on;

subplot(3, 1, 2);
plot(t_values, q_dot_values, 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Velocidade (rad/s)');
title('Velocidade da Junta');
grid on;

subplot(3, 1, 3);
plot(t_values, q_ddot_values, 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Aceleração (rad/s^2)');
title('Aceleração da Junta');
grid on;

% Commande cinematique
% Matrice Jacobienne
function J = calcular_jacobiana(theta, alpha, r, d)
    num_juntas = length(theta);
    T = eye(4);

    Jv = zeros(3, num_juntas);  % Parte linear da Jacobiana
    Jw = zeros(3, num_juntas);  % Parte angular da Jacobiana

    % Posição e orientação iniciais
    z = [0; 0; 1];  % Eixo Z inicial
    p = [0; 0; 0];  % Origem inicial

    for i = 1:num_juntas
        T_i = dh_modified(alpha(i), r(i), theta(i), d(i));
        T = T * T_i;

        % Posição e eixo da junta atual
        p_i = T(1:3, 4); % Posição da ponta
        z_i = T(1:3, 3); % Eixo Z atual

        % Calcular Jv e Jw para a junta atual
        Jv(:, i) = cross(z, (p_i - p)); % Velocidade linear
        Jw(:, i) = z;                   % Velocidade angular

        z = z_i;
        p = p_i;
    end

    J = [Jv; Jw];
end


theta_atual = [0; pi/4; pi/2; 0; pi/4; pi/3];

J = calcular_jacobiana(theta, alpha, r, d);

X_dot_d = [0.1; 0; 0; 0; 0.1; 0.1]; % Velocidade desejada [vx; vy; vz; wx; wy; wz]

J_plus = pinv(J);

q_dot = J_plus * X_dot_d;

disp('Jacobiana J:');
disp(J);
disp('Pseudoinversa da Jacobiana J+:');
disp(J_plus);
disp('Velocidades articulares q_dot para seguir a velocidade desejada:');
disp(q_dot);

