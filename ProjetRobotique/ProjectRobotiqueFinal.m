P_d = [200; 150; 300]; % Ponto final (input)
P_d_dot = [0.8; 0.8; 0.8]; % Velocidade desejada no espaço cartesiano (vetor 3D) (input)
K_P = 1.5; % Ganho proporcional, ainda nao sei se ta certo

% Parametros da transformacao DH
alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
r = [159, 0, 0, 258, 0, 123];
theta = [0; -1.4576453; -0.898549163; 0; 0; pi];
d = [0, 0, 265.69, 30, 0, 0];

% Parâmetros de controle
tolerance = 50; % Tolerância para o erro de posição
delta_t = 0.01; % Intervalo de tempo para a integração

max_iterations = 1000; % Limite máximo de iterações

iteration = 0; % Contador de iterações

while iteration < max_iterations
    iteration = iteration + 1;
    % Calcular posição atual e Jacobiana
    [joint_positions, P_e] = MGD(alpha, r, theta, d);
    J = Jacobiana(theta, alpha, r, d);
    
    % Calcular erro de posição
    epsilon_p = P_d - P_e;
    epsilon_P = norm(epsilon_p);
    disp('Numero de interacoes: ');
    disp(iteration);
    disp('Taxa de erro: ');
    disp(epsilon_P);
    if epsilon_P < tolerance
        break; % Sai do loop se o erro for pequeno o suficiente
    end

    % Calcular a velocidade articular necessária
    % q_dot = J^+ * (P_d_dot + K * (P_d - P_e))
    q_dot = Position(epsilon_P, P_d_dot, K_P, J);

    % Atualizar ângulos articulares usando integração de Euler
    theta = theta + q_dot * delta_t;
    
    % Exibir a posição atual do efetuador
    disp('Posição atual do efetuador (P_e):');
    disp(P_e);
end

disp('Posição final alcançada:');
disp(P_e);


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
function Param = CalculeTrapeze(robot,qi,qf,duree)	%Calculer les paramètres du trapèze

    q_dot_max = robot(:,1);
    q_ddot_max = robot(:,2);
    t1 = zeros(length(robot),1);
    t2 = zeros(length(robot),1);
    tf = zeros(length(robot),1);

    %Calculer le temps minimum (L'articulation plus lente)
    for i = 1:length(robot)
        t1(i,:) = q_dot_max(i,:)/q_ddot_max(i,:);
        t2(i,:) = (qf(i) - qi(i))/q_dot_max(i,:);
        tf(i,:) = t1(i,:) + t2(i,:);
    end 
    t1a = t1; 
    tfa = tf; %Valeurs originales
    TempsMin = max(tf);

    %Calculer des nouvelles valeurs de vitesse et d'accélération de chaque articulation
    q_dot = zeros(length(robot),1);
    q_ddot = zeros(length(robot),1);

    if duree <= TempsMin %Toutes les articulations se déplacent ensemble avec la même durée (temps minimum)
        for j = 1:length(robot)
            tf(j,:) = TempsMin;
            q_dot(j,:) = (q_dot_max(j,:)*tfa(j,:))/tf(j,:); 
            t1(j,:) = (q_dot_max(j,:)*t1a(j,:))/q_dot(j,:); %nouvelles valeurs de t1 et t2
            t2(j,:) = tf(j,:) - t1(j,:);
            q_ddot(j,:) = q_dot(j,:)/t1(j,:);
        end
    else
        for j = 1:length(robot)
            tf(j,:) = duree;
            q_dot(j,:) = (q_dot_max(j,:)*tfa(j,:))/tf(j,:);
            t1(j,:) = (q_dot_max(j,1)*t1a(j,:))/q_dot(j,:);
            t2(j,:) = tf(j,:) - t1(j,:);
            q_ddot(j,:) = q_dot(j,:)/t1(j,:);
        end
    end

    Param = [qi qf t1 t2 tf q_dot q_ddot];
end


function [q, V, A] = CalculeQ(robot, Param, t)
    % Desestruturação dos parâmetros
    qi = Param(:,1);
    qf = Param(:,2);
    t1 = Param(:,3);
    t2 = Param(:,4);
    tf = Param(:,5);
    q_dot = Param(:,6);
    q_ddot = Param(:,7);

    % Inicialização dos resultados
    q = zeros(length(robot),1,length(t));
    qt1 = zeros(length(robot),1);
    qt2 = zeros(length(robot),1);
    V = zeros(length(robot),1,length(t));
    A= zeros(length(robot),1,length(t));


    % Verificar em qual fase o tempo t está e calcular posição, velocidade e aceleração
    for j = 1:length(t)
        for i = 1:length(robot)
            qt1(i,:) = qi(i,:) + (q_ddot(i,:)*t1(i,:)^2)/2;
            qt2(i,:) = qt1(i,:) + q_dot(i,:)*(t2(i,:)-t1(i,:));
   
            if t(j) <= t1(i,:)
		    % Fase de aceleração
                q(i,1,j) = qi(i,:) + (q_ddot(i,:)*t(j)^2)/2;
	    	    V(i,1,j) = q_ddot(i,:) * t(j);
        	    A(i,1,j) = q_ddot(i,:);
            elseif t(j) > t1(i,:) && t(j) <= t2(i,:)
		    % Fase de velocidade constante
                q(i,1,j) = qt1(i,:) + q_dot(i,:)*(t(j) - t1(i,:));
		        V(i,1,j) = q_dot(i,:);
        	    A(i,1,j) = 0;
            elseif t(j) > t2(i,:) && t(j) <= tf(i,:)
		    % Fase de desaceleração
                q(i,1,j) = qt2(i,:) + q_dot(i,:)*(t(j) - t2(i,:)) - (q_ddot(i,:)*(t(j) - t2(i,:))^2)/2;
		        V(i,1,j) = q_ddot(i,:) * (tf(i,:) - t(j));
        	    A(i,1,j) = -q_ddot(i,:);
            else
		    % Após tf, posição é exatamente qf
                q(i,1,j) = qf(i,:);
		        V(i,1,j) = 0;
        	    A(i,1,j) = 0;
            end
        end
    end    
end

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

