clear all
dt = 0.01;

disp('Open Vrep Api............');
[vrep, clientID] = VrepOpenApi();

% Configurar o modo síncrono
vrep.simxSynchronous(clientID, true);
vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, dt, vrep.simx_opmode_oneshot);
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait);

% Recuperar handles das juntas
disp('Retrieving all the joint handles.......');
for idobj = 1:6
    objectName = strcat('joint', num2str(idobj));
    listObjects(idobj).name = objectName;
end
[all_ok, listObjects] = VrepGetHandles(vrep, clientID, listObjects);

if all_ok == false
   disp('An error occurred while retrieving the object handles ==> stop simulation');
   return;
end

% Parâmetros de controle
P_d = [200; 150; 300]; % Ponto final desejado
P_d_dot = [0.8; 0.8; 0.8]; % Velocidade desejada no espaço cartesiano
A_d = [-0.7071 0 0.7071; 0 -1 0; 0.7071 0 -0.7071]; % Orientação final desejada
W_d = [0.5; 0.5; 0.5]; % Velocidade angular desejada
K_P = 2.5; % Ganho proporcional para posição
K_0 = 2.5; % Ganho proporcional para orientação

% Parâmetros da transformação DH
alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
r = [159, 0, 0, 258, 0, 123];
theta = [0; -1.4576453; -0.898549163; 0; 0; pi];
d = [0, 0, 265.69, 30, 0, 0];

% Parâmetros da Lei do Trapézio
v_max = [3.3; 3.3; 3.3; 3.3; 3.2; 3.2];
a_max = [30; 30; 30; 30; 30; 30];
robot = [v_max a_max];
qi = theta; % Posição inicial
qf = [pi; pi/2; 3*pi/4; pi; pi/2; pi]; % Posição final
duree = 1;

% Calcular os parâmetros do perfil trapezoidal
Param = CalculeTrapeze(robot, qi, qf, duree);

% Inicializar variáveis de controle
tolerance = norm([0.005; 0.005; 0.005]);
toleranceOri = norm([0.005; 0.005; 0.005]);
delta_t = dt;
iteration = 0;
max_iterations = 1000;

% Loop principal de controle
while iteration < max_iterations
    iteration = iteration + 1;

    % Calcular posição atual e Jacobiana
    [joint_positions, joint_orientations, P_e, A_e] = MGD(alpha, r, theta, d);
    J = Jacobiana(theta, alpha, r, d);

    % Calcular erro de posição e orientação
    epsilon_p = P_d - P_e;
    epsilon_P = norm(epsilon_p);
    A = A_d * A_e.';
    epsilon_0 = 0.5 * [A(3,2) - A(2,3); A(1,3) - A(3,1); A(2,1) - A(1,2)];
    epsilon_0_n = norm(epsilon_0);

    % Verificar se o erro está dentro da tolerância
    if epsilon_P < tolerance && epsilon_0_n < toleranceOri
        break;
    end

    % Calcular a matriz L e a velocidade angular W_e
    L = -0.5 * (vector2matrix(A_e(:,1))*vector2matrix(A_d(:,1)) + vector2matrix(A_e(:,2))*vector2matrix(A_d(:,2)) + vector2matrix(A_e(:,3))*vector2matrix(A_d(:,3)));
    W_e = L \ (L.' * W_d + K_0 * epsilon_0);

    % Calcular a velocidade articular necessária
    q_dot = Position(epsilon_P, P_d_dot, K_P, J, W_e);

    % Aplicar a Lei do Trapézio para limitar as velocidades articulares
    [q_values, ~, ~] = CalculeQ(robot, Param, iteration * delta_t);
    q_dot_limited = min(q_dot, q_values);

    % Atualizar ângulos articulares (integração de Euler)
    theta = theta + q_dot_limited * delta_t;

    % Enviar as posições calculadas para as juntas no V-REP
    vrep.simxPauseCommunication(clientID, 1);
    for j = 1:6
        vrep.simxSetJointTargetPosition(clientID, listObjects(j).handle, theta(j), vrep.simx_opmode_oneshot);
    end
    vrep.simxPauseCommunication(clientID, 0);

    % Avançar um passo de simulação
    vrep.simxSynchronousTrigger(clientID);

    % Ler e exibir a posição atual de cada articulação (opcional)
    for j = 1:6
        [err, theta(j)] = vrep.simxGetJointPosition(clientID, listObjects(j).handle, vrep.simx_opmode_oneshot_wait);
    end
end

% Finalizar a simulação
pause(2)
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(clientID);
vrep.delete();
