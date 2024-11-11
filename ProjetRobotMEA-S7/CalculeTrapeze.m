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

