
%--------------------------------------------------------------------------
% Este método usa os coeficientes da equação de um elipsoide para calcular
% uma matriz C e um vetor o. Esses parâmetros podem ser utilizados para 
% transformar o elipsoide em uma esfera aplicando a equacao p' = C*p + o
% em cada ponto do elipsoide.
%--------------------------------------------------------------------------

function [C, o] = Quadric2Calibration(Coeficientes)

    %Passando os coeficientes para a forma matricial ----------------------
    a = Coeficientes(1);
    b = Coeficientes(2);
    c = Coeficientes(3);
    d = Coeficientes(4);
    e = Coeficientes(5);
    f = Coeficientes(6);
    g = Coeficientes(7);
    h = Coeficientes(8);
    i = Coeficientes(9);
    j = Coeficientes(10);

    A = [  a, d/2, e/2;
         d/2,   b, f/2;
         e/2, f/2,   c ];

    B = [g, h, i];

    
    % Calculando os parâmetros de calibração ------------------------------
    [V, D] = eig(A);
    invC = V * sqrt(inv(D)) * V';
    Caux = inv( invC );
    oaux = (B * invC / 2)';


    % Normalizando para que a esfera fique com raio unitário --------------
    jc = oaux' * oaux - B * invC * oaux + j;
    norm = sqrt(abs(jc));
    C = Caux / norm;
    o = oaux / norm;

end