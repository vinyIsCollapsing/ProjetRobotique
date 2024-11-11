function [M] = vector2matrix(V)
    % Function qui associe au vector V(3,1) la matrice M(3,3)
    M = [0 -V(3) V(2); V(3) 0 -V(1); -V(2) V(1) 0];
end