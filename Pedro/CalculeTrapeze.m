function Param = CalculeTrapeze(robot,qi,qf,duree)
%Calculer les paramètres du trapèze
t1 = zeros(length(robot),1);
t2 = zeros(length(robot),1);
tf = zeros(length(robot),1);

%Calculer le temps minimum (L'articulation plus lente)
for i = 1:length(robot)
    t1(i,:) = robot(i,1)/robot(i,2);
    t2(i,:) = (qf(i) - qi(i))/robot(i,1);
    tf(i,:) = t1(i,:) + t2(i,:);
end 
t1a = t1; 
tfa = tf; %Valeurs originales
TempsMin = max(tf);

%Calculer des nouvelles valeurs de vitesse et d'accélération de chaque articulation
V = zeros(length(robot),1);
Ac = zeros(length(robot),1);

if duree <= TempsMin %Toutes les articulations se déplacent ensemble avec la même durée (temps minimum)
    for j = 1:length(robot)
        tf(j,:) = TempsMin;
        V(j,:) = (robot(j,1)*tfa(j,:))/tf(j,:); 
        t1(j,:) = (robot(j,1)*t1a(j,:))/V(j,:); %nouvelles valeurs de t1 et t2
        t2(j,:) = tf(j,:) - t1(j,:);
        Ac(j,1) = V(j,:)/t1(j,:);
    end
else
    for j = 1:length(robot)
        tf(j,:) = duree;
        V(j,:) = (robot(j,1)*tfa(j,:))/tf(j,:);
        t1(j,:) = (robot(j,1)*t1a(j,:))/V;
        t2(j,:) = tf(j,:) - t1(j,:);
        Ac(j,1) = V(j,:)/t1(j,:);
    end
end

Param = [qi qf t1 t2 tf V Ac];
end

