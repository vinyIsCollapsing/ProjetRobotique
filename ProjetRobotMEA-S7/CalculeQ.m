function [q, V, A] = CalculeQ(robot, Param, t)
    % Déstructuration des paramètres
    qi = Param(:,1);
    qf = Param(:,2);
    t1 = Param(:,3);
    t2 = Param(:,4);
    tf = Param(:,5);
    q_dot = Param(:,6);
    q_ddot = Param(:,7);

    % Initialisation des résultats
    q = zeros(length(robot),1,length(t));
    qt1 = zeros(length(robot),1);
    qt2 = zeros(length(robot),1);
    V = zeros(length(robot),1,length(t));
    A= zeros(length(robot),1,length(t));

    % Vérification de la phase temporelle t et calcul de la position, vitesse et accélération
    for j = 1:length(t)
        for i = 1:length(robot)
            qt1(i,:) = qi(i,:) + (q_ddot(i,:)*t1(i,:)^2)/2;
            qt2(i,:) = qt1(i,:) + q_dot(i,:)*(t2(i,:)-t1(i,:));
   
            if t(j) <= t1(i,:)
		        % Phase d'accélération
                q(i,1,j) = qi(i,:) + (q_ddot(i,:)*t(j)^2)/2;
	    	    V(i,1,j) = q_ddot(i,:) * t(j);
        	    A(i,1,j) = q_ddot(i,:);
            elseif t(j) > t1(i,:) && t(j) <= t2(i,:)
		        % Phase de vitesse constante
                q(i,1,j) = qt1(i,:) + q_dot(i,:)*(t(j) - t1(i,:));
		        V(i,1,j) = q_dot(i,:);
        	    A(i,1,j) = 0;
            elseif t(j) > t2(i,:) && t(j) <= tf(i,:)
		        % Phase de décélération
                q(i,1,j) = qt2(i,:) + q_dot(i,:)*(t(j) - t2(i,:)) - (q_ddot(i,:)*(t(j) - t2(i,:))^2)/2;
		        V(i,1,j) = q_ddot(i,:) * (tf(i,:) - t(j));
        	    A(i,1,j) = -q_ddot(i,:);
            else
		        % Après tf, la position est exactement qf
                q(i,1,j) = qf(i,:);
		        V(i,1,j) = 0;
        	    A(i,1,j) = 0;
            end
        end
    end    
end
