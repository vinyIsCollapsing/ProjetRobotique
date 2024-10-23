clc;
clear;

% Valeurs initiales et matricielles
alpha = [0.0, -pi/2, 0.0, -pi/2, pi/2, -pi/2];
d = [0.0, 0.0, 265.69, 30.0, 0.0, 0.0];
theta = 0.0;
thetaEnsemble = [theta, theta - 1.4576453, theta - 0.898549163, theta, theta, theta, 180];
r = [159.0, 0.0, 0.0, 258.0, 0.0, 123.0];

axes = zeros(3,1,length(d)+1);
orientation = zeros(3,3,length(d)+1);

% Matrice de transition TH
T = zeros(4,4,length(d));
for i = 1 : length(d)
    T(:,:,i) = [cos(thetaEnsemble(i)) -sin(thetaEnsemble(i)) 0 d(i);
    cos(alpha(i))*sin(thetaEnsemble(i)) cos(alpha(i))*cos(thetaEnsemble(i)) -sin(alpha(i)) -r(i)*sin(alpha(i));
    sin(alpha(i))*sin(thetaEnsemble(i)) sin(alpha(i))*cos(thetaEnsemble(i)) cos(alpha(i)) r(i)*cos(alpha(i));
    0 0 0 1];

    % Correction pour sauvegarder les operations matricielles
    if i >= 2
        T(:,:,i) = T(:,:,i - 1) * T(:,:,i);
    end
    
    axes(:,:,i+1) = T(1:3,4:end,i);
    orientation(:, :, i+1) = T(1:3, 1:3, i);
end

% Tracage du squelette
t = 1 : length(d) + 1;
plot3(axes(1,t), axes(2,t), axes(3,t), 'ko-', 'LineWidth', 1);
hold on;

% Extraction des valeurs pour les fleches d'orientation
scale = 100;

for i = 1 : length(d) 
    xVec = orientation(:,1,i+1);
    yVec = orientation(:,2,i+1);
    zVec = orientation(:,3,i+1);

    vertexPosition = axes(:,i+1);

    % Tracez les fleches pour x, y, z
    quiver3(vertexPosition(1), vertexPosition(2), vertexPosition(3), xVec(1), xVec(2), xVec(3), scale, 'r', 'LineWidth', 2.0);
    quiver3(vertexPosition(1), vertexPosition(2), vertexPosition(3), yVec(1), yVec(2), yVec(3), scale, 'g', 'LineWidth', 2.0);
    quiver3(vertexPosition(1), vertexPosition(2), vertexPosition(3), zVec(1), zVec(2), zVec(3), scale, 'b', 'LineWidth', 2.0);
    % quiver3(axes(1,i), axes(2,i), axes(3,i), orientation(1:end, 1, i), orientation(1:end, 2, i), orientation(1:end, 3, i));
end

axis ([-750 750 -750 750 0 800]);
xlabel('x(t)')
ylabel('y(t)')
zlabel('z(t)')
grid on

% Definir le controle
% Calcul du mouvement par la loi du trapeze
function Param = CalculeTrapeze(robot, qi, qf, duree)
    qDotMax = robot.qDotMax;        % Vitesse
    qDotDotMax = robot.qDotDotMax;  % Acceleration

    deltaQ = abs(qf - qi);          % (Déplacement (Deslocamento)

    if duree == 0
        tf = (deltaQ + qDotMax^2 / qDotDotMax) / qDotMax;
    else
        tf = duree;
    end

    t1 = qDotMax / qDotDotMax;
    t3 = t1;
    t2 = tf - 2 * t1;
    
    Param.qi = qi;
    Param.qf = qf;
    Param.tf = tf;
    Param.t1 = t1;
    Param.t2 = t2;
    Param.t3 = t3;
end

function q = CalculeQ(robot, Param, t)
    t1 = Param.t1;
    t2 = Param.t2;
    tf = Param.tf;
    qi = Param.qi;
    qf = Param.qf;
    qDotMax = robot.qDotMax;
    qDotDotMax = robot.qDotDotMax;
    
    % t1 -> tempo de aceleracao
    % t2 -> tempo de constancia
    % t3 -> tempo de desaceleracao
    % tf = t1 + t2 + t3
    if t <= t1
        % Acelerando
        q = qi + 0.5 * qDotDotMax * (t^2);
    elseif t <= t1 + t2
        % Constante
        q = qi + qDotMax * (t - 0.5 * t1);
    else
        % Desaceleracao
        q = qf - 0.5 * qDotDotMax * ((tf - t)^2);
    end
end

% Parameteurs
robot.qDotMax = [3.3, 3.3, 3.3, 3.3, 3.2, 3.2];
robot.qDotDotMax = [30, 30, 30, 30, 30, 30];
qLimites = [-pi, pi; -pi/2, pi/2; -pi/2, 3*pi/4; -pi, pi; -pi/2, pi/2; -pi, pi];

tempSim = linspace(0, 10, 100);

qi = [0 0 0 0 0 0];
qf = [pi/2 pi/4 pi/6 pi/3 pi/5 pi/2];
duree = 5;

for i = 1 : length(qi)
    Param(i) = CalculeTrapeze(robot, qi(i), qf(i), duree);
end 

q_sim = zeros(length(tempSim), length(qi));

t = 1;
junta = 1;
q_valor = CalculeQ(robot, Param(junta), t);
disp(['Valor de q para a junta ', num2str(junta), ' no tempo ', num2str(t), ' é: ', num2str(q_valor)]);

for i = 1 : length(tempSim)
    t = tempSim(i);
    for j = 1:length(qi)
        q_valor = CalculeQ(robot, Param(j), t);
        q_sim(i, j) = q_valor(1, j);
    end
end

figure;
plot(tempSim, q_sim, 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Posicao articular (rad/s)')
grid on;