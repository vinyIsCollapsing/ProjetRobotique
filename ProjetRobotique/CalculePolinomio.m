function [q, q_dot, q_ddot] = CalculePolinomio(qi, qf, tf, t)
    % Calcular os coeficientes da função cúbica para trajetória polinomial
    a0 = qi;
    a1 = 0;
    a2 = (3 * (qf - qi)) / tf^2;
    a3 = (-2 * (qf - qi)) / tf^3;

    % Calcular posição, velocidade e aceleração em t
    q = a0 + a1 * t + a2 * t^2 + a3 * t^3;
    q_dot = a1 + 2 * a2 * t + 3 * a3 * t^2;
    q_ddot = 2 * a2 + 6 * a3 * t;
end