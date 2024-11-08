function q = CalculeQ(robot,Param,t)
% Permet de calculer les positions articulaires en fonction du temps

q = zeros(length(robot),1,length(t));
qt1 = zeros(length(robot),1);
qt2 = zeros(length(robot),1);


for j = 1:length(t)
    for i = 1:length(robot)
        qt1(i,1) = Param(i,1) + (Param(i,7)*Param(i,3)^2)/2;
        qt2(i,1) = qt1(i,1) + Param(i,6)*(Param(i,4)-Param(i,3));
   

        if t(j) <= Param(i,3)
            q(i,1,j) = Param(i,1) + (Param(i,7)*t(j)^2)/2;
        elseif t(j) > Param(i,3) && t(j) <= Param(i,4)
            q(i,1,j) = qt1(i,1) + Param(i,6)*(t(j) - Param(i,3));
        elseif t(j) > Param(i,4) && t(j) <= Param(i,5)
            q(i,1,j) = qt2(i,1) + Param(i,6)*(t(j) - Param(i,4)) - (Param(i,7)*(t(j) - Param(i,4))^2)/2;
        else
            q(i,1,j) = Param(i,2);
        end
    end
end






%r = zeros(length(robot),1);
%q = zeros(length(robot),1);

%if t <= max(Param(:,5))
%    for i = 1:length(Param)
%        r(i,1) = 3*(t/Param(i,5))^2 - 2*(t/Param(i,5))^3;
%        q(i,1) = Param(i,1) + r(i,:)*(Param(i,2)-Param(i,1));
%    end
%else
%    if t > max(Param(:,5))
%        q(:,1) = Param(:,2);
%    end
%end



