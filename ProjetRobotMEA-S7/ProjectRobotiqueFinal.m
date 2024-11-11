% Paramètres d'entrée (position)
P_d = [200; 100; 600]; % Point final (entrée)
P_d_dot = [0.8; 0.8; 0.8]; % Vitesse désirée dans l'espace cartésien (vecteur 3D) (entrée)
K_P = 2.5; % Gain proportionnel, pas encore certain de sa valeur

% Paramètres d'entrée (orientation)
A_d = [-0.7071 0 0.7071; 0 -1 0; 0.7071 0 -0.7071]; % Orientation finale (entrée)
W_d = [0.5; 0.5; 0.5]; % Vitesse angulaire désirée (entrée)
K_0 = 2.5;

% Paramètres de la transformation DH
alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
r = [159, 0, 0, 258, 0, 123];
theta = [0; -1.4576453; -0.898549163; 0; 0; pi];
d = [0, 0, 265.69, 30, 0, 0];

% Définition des paramètres
v_max = [3.3; 3.3; 3.3; 3.3; 3.2; 3.2];
a_max = [30; 30; 30; 30; 30; 30];
robot = [v_max a_max];
qi = [-pi; -pi/2; -pi/2; -pi; -pi/2; -pi];  % Position initiale
qf = [pi; pi/2; 3*pi/4; pi; pi/2; pi];      % Position finale
duree = 1;

% Calculer les paramètres du profil trapézoïdal
Param = CalculeTrapeze(robot, qi, qf, duree);

% Initialiser les variables de contrôle
tolerance = norm([0.005; 0.005; 0.005]); % Tolérance pour l'erreur de position
toleranceOri = norm([0.005; 0.005; 0.005]); % Tolérance pour l'erreur d'orientation
delta_t = 0.01; % Intervalle de temps pour l'intégration
iteration = 0;
max_iterations = 1000;

% Boucle principale de contrôle
while iteration < max_iterations
    % Calculer la position actuelle et la Jacobienne
    [joint_positions, joint_orientations, P_e, A_e] = MGD(alpha, r, theta, d, iteration);
    J = Jacobien(theta, alpha, r, d);
    
    % Calculer l'erreur de position
    epsilon_p = P_d - P_e;
    epsilon_P = norm(epsilon_p);
    % Calculer l'erreur d'orientation
    A = A_d * A_e.';
    epsilon_0 = 0.5 * [A(3,2) - A(2,3); A(1,3) - A(3,1); A(2,1) - A(1,2)];
    epsilon_0_n = norm(epsilon_0);
    
    if epsilon_P < tolerance || epsilon_0_n < toleranceOri
        break; % Quitter la boucle si l'erreur est suffisamment faible
    end

    disp('Nombre d´itérations : ');
    disp(iteration);
    disp('Taux d´erreur : ');
    disp(epsilon_P);
    
    % Calculer la matrice L
    L = -0.5 * (vector2matrix(A_e(:,1))*vector2matrix(A_d(:,1))+vector2matrix(A_e(:,2))*vector2matrix(A_d(:,2))+vector2matrix(A_e(:,3))*vector2matrix(A_d(:,3)));
    
    % Calculer la vitesse angulaire instantanée
    W_e = inv(L) * (L.' * W_d + K_0 * epsilon_0);

    % Calculer la vitesse articulaire nécessaire
    % q_dot = J^+ * (P_d_dot + K * (P_d - P_e))
    q_dot_desired = Position(epsilon_P, P_d_dot, K_P, J, W_e);

    % Temps actuel pour le calcul trapézoïdal
    t_current = iteration * delta_t;
    
    % Obtenir la position, vitesse et accélération limitées par le profil trapézoïdal
    [q, V, A] = CalculeQ(robot, Param, t_current);

    % Limiter la vitesse articulaire `q_dot` en fonction du profil trapézoïdal
    q_dot_limited = min(abs(q_dot_desired), abs(V(:,1,1))) .* sign(q_dot_desired);

    % Mettre à jour les angles articulaires (intégration d'Euler) avec la vitesse limitée
    theta = theta + q_dot_limited * delta_t;
    
    % Afficher la position actuelle de l'effecteur pour le suivi
    disp('Position actuelle de l´effecteur (P_e):');
    disp(P_e);
    disp('Position désirée (P_d):');
    disp(P_d);
    disp('Erreur de position:');
    disp(epsilon_P);

    iteration = iteration + 1;
end

disp('Position finale atteinte:');
disp(P_e);
disp('Orientation finale atteinte:');
disp(A_e);
disp('Position articulaire finale:');
disp(theta);

% Commande articulaire
% Loi Trapèze
% Définir les limites du robot comme une structure
v_max = [3.3 3.3 3.3 3.3 3.2 3.2];
a_max = [30 30 30 30 30 30];

robot = [v_max' a_max'];

% Paramètres initiaux
qi = [-pi; -pi/2; -pi/2; -pi; -pi/2; -pi];	% Position initiale (rad)
qf = [pi; pi/2; 3*pi/4; pi; pi/2; pi];      % Position finale (rad)  
     
duree = 0;       % Temps total souhaité (s)
N = 6;		   % Articulation souhaitée

% Calculer les paramètres de la trajectoire trapézoïdale
Param = CalculeTrapeze(robot, qi, qf, duree);

% Calcul de la trajectoire au fil du temps
t_values = linspace(0, Param(N,5), 2000);  % Vecteur de temps de 0 à tf avec 100 points

[Q, V, A] = CalculeQ(robot, Param, t_values);

q_values = squeeze(Q(1,1,:));       % Vecteur pour stocker les positions de l'articulation
q_dot_values = squeeze(V(1,1,:));   % Vecteur pour stocker les vitesses
q_ddot_values = squeeze(A(1,1,:));  % Vecteur pour stocker les accélérations

% Tracer la position de l'articulation au fil du temps
figure;
subplot(3, 1, 1);
plot(t_values, q_values, 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Position (rad)');
title('Position de l´articulation');
grid on;

% Tracer la vitesse de l'articulation au fil du temps
subplot(3, 1, 2);
plot(t_values, q_dot_values, 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Vitesse (rad/s)');
title('Vitesse de l´articulation');
grid on;

% Tracer l'accélération de l'articulation au fil du temps
subplot(3, 1, 3);
plot(t_values, q_ddot_values, 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Accélération (rad/s²)');
title('Accélération de l´articulation');
grid on;

% Commande articulaire
% Loi Polynomiale
% Paramètres initiaux
qi = -pi;          % Position initiale (rad)
qf = pi;       % Position finale (rad)
tf = 2;          % Temps total souhaité (s)

% Calcul de la trajectoire au fil du temps
t_values = linspace(0, tf, 100);
q_values = zeros(size(t_values)); 
q_dot_values = zeros(size(t_values)); 
q_ddot_values = zeros(size(t_values)); 

for i = 1:length(t_values)
    [q_values(i), q_dot_values(i), q_ddot_values(i)] = CalculePolynomiale(qi, qf, tf, t_values(i));
end

% Tracer la position, vitesse et accélération au fil du temps
figure;

subplot(3, 1, 1);
plot(t_values, q_values, 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Position (rad)');
title('Position de l´articulation');
grid on;

subplot(3, 1, 2);
plot(t_values, q_dot_values, 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Vitesse (rad/s)');
title('Vitesse de l´articulation');
grid on;

subplot(3, 1, 3);
plot(t_values, q_ddot_values, 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Accélération (rad/s²)');
title('Accélération de l´articulation');
grid on;
