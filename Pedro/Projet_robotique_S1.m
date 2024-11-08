%Robot parameters
alpha = [0 -pi/2 0 -pi/2 pi/2 -pi/2];
d = [0 0 265.69 30 0 0];
theta = [0 -1.4576453 -0.898549163 0 0 180];
r = [159 0 0 258 0 123];
v_max = [3.3 3.3 3.3 3.3 3.2 3.2];
a_max = [30 30 30 30 30 30];
qi = [-pi; -pi/2; -pi/2; -pi; -pi/2; -pi];
qf = [pi; pi/2; 3*pi/4; pi; pi/2; pi];
robot = [v_max' a_max'];

%Variables used
T = zeros(4,4,length(alpha));
X = zeros(1,length(alpha)+1);
Y = zeros(1,length(alpha)+1);
Z = zeros(1,length(alpha)+1);
I = zeros(3,1,length(alpha)+1);
J = zeros(3,1,length(alpha)+1);
K = zeros(3,1,length(alpha)+1);
TF = 1;
Rot = zeros(3,3,length(alpha));

%Calculating the transformation matrices
for i = 1:length(alpha)
    T(:,:,i) = [cos(theta(i)) -sin(theta(i)) 0 d(i);
                cos(alpha(i))*sin(theta(i)) cos(alpha(i))*cos(theta(i)) -sin(alpha(i)) -r(i)*sin(alpha(i));
                sin(alpha(i))*sin(theta(i)) sin(alpha(i))*cos(theta(i)) cos(alpha(i)) r(i)*cos(alpha(i));
                0 0 0 1];
    TF = TF*T(:,:,i);
    Rot(:,:,i) = TF(1:3,1:3);
    X(1,i+1) = TF(1,4);
    Y(1,i+1) = TF(2,4);
    Z(1,i+1) = TF(3,4);
    I(:,:,i+1) = Rot(:,1,i); %Points for plotting each articulation axis
    J(:,:,i+1) = Rot(:,2,i);
    K(:,:,i+1) = Rot(:,3,i);
end

%Plotting the robot arms with the articulations 
X(1,1) = 0;
Y(1,1) = 0;
Z(1,1) = 0;
plot3(X,Y,Z,'-o','LineWidth',3)
xlabel('X')
ylabel('Y')
zlabel('Z')
hAxis=gca;
hAxis.XRuler.FirstCrossoverValue  = 0; % X crossover with Y axis
hAxis.XRuler.SecondCrossoverValue  = 0; % X crossover with Z axis
hAxis.YRuler.FirstCrossoverValue  = 0; % Y crossover with X axis
hAxis.ZRuler.FirstCrossoverValue  = 0; % Z crossover with X axis
axis([-800 800 -800 800 0 800])
grid
hold on

%Plotting each articulation axis
I(:,:,1) = [1; 0; 0]; 
J(:,:,1) = [0; 1; 0];
K(:,:,1) = [0; 0; 1];
for j = 1:length(X)
    quiver3(X(:,j), Y(:,j), Z(:,j), I(1,:,j), J(1,:,j), K(1,:,j), 100, 'Color','r');
    quiver3(X(:,j), Y(:,j), Z(:,j), I(2,:,j), J(2,:,j), K(2,:,j), 100, 'Color','g');
    quiver3(X(:,j), Y(:,j), Z(:,j), I(3,:,j), J(3,:,j), K(3,:,j), 100, 'Color','b');
end

%Jacobienne
qtheta = [0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868];
P = [-0.3668886207137053 -0.03791609269591508 0.8634137719059174]';
Jac = Jacobbienne(qtheta,P);

%Commande par Loi Trapeze
duree = 0;
Trapeze = CalculeTrapeze(robot,qi,qf,duree);
Q = CalculeQ(robot,Trapeze,2.15);

%Plot de la position Q en fonction du temps
t = linspace(0,4,1000);
Qt = CalculeQ(robot,Trapeze,t);
P = squeeze(Qt(1,1,:));
figure
plot(t,P)
