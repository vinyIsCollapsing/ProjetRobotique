% Parametros de input (posicao)
P_d = [200; 150; 300]; % Ponto final (input)
P_d_dot = [0.8; 0.8; 0.8]; % Velocidade desejada no espaço cartesiano (vetor 3D) (input)
K_P = 1.5; % Ganho proporcional, ainda nao sei se ta certo

% Parametros de input (orientacao)
A_d = [-0.7071 0 0.7071; 0 -1 0; 0.7071 0 -0.7071]; % Orientação final (input)
W_d = [0.5; 0.5; 0.5]; % Velocidade angular desejada (input)
K_0 = 1.5;

% Parametros da transformacao DH
alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
r = [159, 0, 0, 258, 0, 123];
theta = [0; -1.4576453; -0.898549163; 0; 0; pi];
d = [0, 0, 265.69, 30, 0, 0];

% Parametros de limite da commande articulaire
% Limites de posição (rad), por exemplo:
q_min = [-pi; -pi/2; -pi/2; -pi; -pi/2; -pi];
q_max = [pi; pi/2; 3*pi/4; pi; pi/2; pi];

% Limites de velocidade (rad/s):
q_dot_max = [3.3; 3.3; 3.3; 3.3; 3.2; 3.2];

% Limites de aceleração (rad/s^2):
q_ddot_max = [30; 30; 30; 30; 30; 30];

% Parâmetros de controle
tolerance = norm([0.005; 0.005; 0.005]); % Tolerância para o erro de posição
toleranceOri = norm([0.005; 0.005; 0.005]); % Tolerância para o erro de rotação (orientacao)
delta_t = 0.01; % Intervalo de tempo para a integração
max_iterations = 1000; % Limite máximo de iterações
iteration = 0; % Contador de iterações

while iteration < max_iterations
    iteration = iteration + 1;
    % Calcular posição atual e Jacobiana
    [joint_positions, joint_orientations, P_e, A_e] = MGD(alpha, r, theta, d);
    J = Jacobiana(theta, alpha, r, d);
    
    % Calcular erro de posição
    epsilon_p = P_d - P_e;
    epsilon_P = norm(epsilon_p);
    disp('Numero de interacoes: ');
    disp(iteration);
    disp('Taxa de erro: ');
    disp(epsilon_P);
    
    % Calcular erro de orientação
    A = A_d * A_e.';
    epsilon_0 = 0.5 * [A(3,2) - A(2,3); A(1,3) - A(3,1); A(2,1) - A(1,2)];
    epsilon_0_n = norm(epsilon_0);

    if epsilon_P < tolerance || epsilon_0_n < toleranceOri
        break; % Sai do loop se o erro for pequeno o suficiente
    end

    % Calcular a matriz L
    L = -0.5 * (vector2matrix(A_e(:,1))*vector2matrix(A_d(:,1))+vector2matrix(A_e(:,2))*vector2matrix(A_d(:,2))+vector2matrix(A_e(:,3))*vector2matrix(A_d(:,3)));
    
    % Calcular a velocidade angular momentânea
    W_e = inv(L) * (L.' * W_d + K_0 * epsilon_0);

    % Calcular a velocidade articular necessária
    % q_dot = J^+ * (P_d_dot + K * (P_d - P_e))
    q_dot = Position(epsilon_P, P_d_dot, K_P, J, W_e);

    % Atualizar ângulos articulares usando integração de Euler
    theta = theta + q_dot * delta_t;
    
    % Exibir a posição atual do efetuador
    disp('Posição atual do efetuador (P_e):');
    disp(P_e);
end

disp('Posição final alcançada:');
disp(P_e);
disp('Orientação final alcançada:');
disp(A_e);
disp('Posicao da junta final:');
disp(q_dot);


% [joint_positions, P_e] = MGD(alpha, r, theta, d);
% disp('Posicao das juntas');
% disp(joint_positions);
% disp('Posicao do ultimo ponto (lapis) (P_e):');
% disp(P_e);

% J = Jacobiana(theta, alpha, r, d);
% disp('Matriz jacobiana (J):');
% disp(J);

% q_dot = Position(P_d, P_e, K_P, P_d_dot, J)
% disp('Velocidade articular (q_dot):');
% disp(q_dot);

% Commande articulaire
% Loi Trapeze
% Definir limites do robô como uma estrutura
v_max = [3.3 3.3 3.3 3.3 3.2 3.2];
a_max = [30 30 30 30 30 30];

robot = [v_max' a_max'];

% Parâmetros iniciais
qi = [-pi; -pi/2; -pi/2; -pi; -pi/2; -pi];	% Posição inicial (rad)
qf = [pi; pi/2; 3*pi/4; pi; pi/2; pi];        % Posição final (rad)  
     
duree = 0;       % Tempo total desejado (s)
N = 6;		 % Articulação desejada

% Calcular parâmetros da trajetória trapézio
Param = CalculeTrapeze(robot, qi, qf, duree);

% Cálculo da trajetória ao longo do tempo
t_values = linspace(0, Param(N,5), 2000);  % Vetor de tempo de 0 até tf com 100 pontos

[Q, V, A] = CalculeQ(robot, Param, t_values);

q_values = squeeze(Q(1,1,:));       % Vetor para armazenar posições da junta
q_dot_values = squeeze(V(1,1,:));   % Vetor para armazenar velocidades
q_ddot_values = squeeze(A(1,1,:));  % Vetor para armazenar acelerações


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

