function Jac = Jacobbienne(q,P)
% l’algorithme calculant la matrice Jacobienne géométrique définie par les paramètres
% q l'angle
% p0 coordoner du point pour la Jacobienne

%Paramètres robo KuKa LWR
alpha = [0 -pi/2 0 -pi/2 pi/2 -pi/2 0];
d = [0 0 265.69 30 0 0 123];
theta = [0 -1.4576453 -0.898549163 0 0 0 -pi];
r = [159 0 0 258 0 0 0];
q = q+theta;

%Variables
Jac = zeros(6,length(alpha));

%Calcul des matrices de transformation + Jacobienne
Taux = 1;
for i = 1:length(alpha)
    T = Taux*[cos(q(i)) -sin(q(i)) 0 d(i);
              cos(alpha(i))*sin(q(i)) cos(alpha(i))*cos(q(i)) -sin(alpha(i)) -r(i)*sin(alpha(i));
              sin(alpha(i))*sin(q(i)) sin(alpha(i))*cos(q(i)) cos(alpha(i)) r(i)*cos(alpha(i)); 
              0 0 0 1];
    aj = T(1:3,3); 
    OP = P - T(1:3,4);
    Jac(:,i) = [cross(aj,OP); aj];
    Taux = T;
end