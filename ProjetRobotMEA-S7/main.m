clear all
dt = 0.01;

disp('Ouverture de API Vrep............');
[vrep, clientID] = VrepOpenApi();

% Configurer le mode synchrone
vrep.simxSynchronous(clientID, true);
vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, dt, vrep.simx_opmode_oneshot);
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait);

% Récupérer les handles des articulations
disp('Récupération de tous les handles des articulations.......');
for idobj = 1:6
    objectName = strcat('joint', num2str(idobj));
    listObjects(idobj).name = objectName;
end
[all_ok, listObjects] = VrepGetHandles(vrep, clientID, listObjects);

if all_ok == false
   disp('Une erreur est survenue lors de la récupération des handles des objets ==> arrêt de la simulation');
   return;
end

% Paramètres de contrôle
P_d = [500; 100; -300]; % Point final désiré
P_d_dot = [0.8; 0.8; 0.8]; % Vitesse désirée dans l'espace cartésien
A_d = [-0.7071 0 0.7071; 0 -1 0; 0.7071 0 -0.7071]; % Orientation finale désirée
W_d = [0.5; 0.5; 0.5]; % Vitesse angulaire désirée
K_P = 2.5; % Gain proportionnel pour la position
K_0 = 2.5; % Gain proportionnel pour l'orientation

% Paramètres de la transformation DH
alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
r = [159, 0, 0, 258, 0, 123];
theta = [0; -1.4576453; -0.898549163; 0; 0; pi];
d = [0, 0, 265.69, 30, 0, 0];

% Paramètres de la Loi du Trapèze
v_max = [3.3; 3.3; 3.3; 3.3; 3.2; 3.2];
a_max = [30; 30; 30; 30; 30; 30];
robot = [v_max a_max];
qi = theta; % Position initiale
qf = [pi; pi/2; 3*pi/4; pi; pi/2; pi]; % Position finale
duree = 2.5;

% Calculer les paramètres du profil trapézoïdal
Param = CalculeTrapeze(robot, qi, qf, duree);

% Initialiser les variables de contrôle
tolerance = norm([0.005; 0.005; 0.005]);
toleranceOri = norm([0.005; 0.005; 0.005]);
delta_t = dt;
iteration = 0;
max_iterations = 1000;

% Boucle principale de contrôle
while iteration < max_iterations
    % Temps actuel pour le calcul trapézoïdal
    t_current = iteration * delta_t;

    % Obtenir la position, vitesse et accélération de la trajectoire désirée
    [q_trapezoidal, V_trapezoidal, ~] = CalculeQ(robot, Param, t_current);

    % Calculer la position actuelle et la Jacobienne
    [joint_positions, joint_orientations, P_e, A_e] = MGD(alpha, r, theta, d, iteration);
    J = Jacobien(theta, alpha, r, d);

    % Calculer l'erreur de position et d'orientation par rapport à la trajectoire désirée
    epsilon_p = P_d - P_e;
    epsilon_P = norm(epsilon_p);
    A = A_d * A_e.';
    epsilon_0 = 0.5 * [A(3,2) - A(2,3); A(1,3) - A(3,1); A(2,1) - A(1,2)];
    epsilon_0_n = norm(epsilon_0);

    % Vérifier si l'erreur est dans la tolérance
    if epsilon_P < tolerance && epsilon_0_n < toleranceOri
        break;
    end

    % Calculer la matrice L et la vitesse angulaire W_e
    L = -0.5 * (vector2matrix(A_e(:,1))*vector2matrix(A_d(:,1)) + vector2matrix(A_e(:,2))*vector2matrix(A_d(:,2)) + vector2matrix(A_e(:,3))*vector2matrix(A_d(:,3)));
    W_e = L \ (L.' * W_d + K_0 * epsilon_0);

    % Calculer la vitesse articulaire nécessaire en fonction du profil trapézoïdal
    q_dot_desired = Position(epsilon_p, V_trapezoidal(1:3,1), K_P, J, W_e);

    % Mettre à jour les angles articulaires (intégration d'Euler) avec la vitesse calculée
    theta = theta + q_dot_desired * delta_t;

    % Envoyer les positions calculées aux articulations dans V-REP
    vrep.simxPauseCommunication(clientID, 1);
    for j = 1:6
        vrep.simxSetJointTargetPosition(clientID, listObjects(j).handle, theta(j), vrep.simx_opmode_oneshot);
    end
    vrep.simxPauseCommunication(clientID, 0);

    % Avancer d'un pas de simulation
    vrep.simxSynchronousTrigger(clientID);

    % Lire et afficher la position actuelle de chaque articulation (optionnel)
    for j = 1:6
        [err, theta(j)] = vrep.simxGetJointPosition(clientID, listObjects(j).handle, vrep.simx_opmode_oneshot_wait);
    end
    iteration = iteration + 1;
end

% Arrêter la simulation
pause(2)
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(clientID);
vrep.delete();
