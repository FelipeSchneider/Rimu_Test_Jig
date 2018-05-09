
% -------------------------------------------------------------------------
% Este método calcula a equacao do elipsoide com minima soma do erro 
% quadratico de distância algébrica em relacao ao conjunto de pontos p.
% Equacao normalizada com a + b + c = 1.
% -------------------------------------------------------------------------

function sol = EllipsoidFitting(p)

    num_pontos = size(p,1);
    x = p(:,1);
    y = p(:,2);
    z = p(:,3);

    % Calculando os somatorios uteis das coordenadas dos pontos -----------
    Q = [x, x.^2, x.^3, x.^4, y, y.^2, y.^3, y.^4, z, z.^2, z.^3, z.^4];

    R = [x.*y, x.^2.*y, x.*y.^2, x.^2.*y.^2, ...
         x.*z, x.^2.*z, x.*z.^2, x.^2.*z.^2, ...
         y.*z, y.^2.*z, y.*z.^2, y.^2.*z.^2 ];

    S = [ x.^3.*y, x.*y.^3, x.^3.*z, x.*z.^3, y.^3.*z, y.*z.^3 ];

    T = [ x.*y.*z, x.^2.*y.*z, x.*y.^2.*z, x.*y.*z.^2];


    q = sum(Q, 1);
    r = sum(R, 1);
    s = sum(S, 1);
    t = sum(T, 1);

    % Transcrevendo para facilitar a leitura ------------------------------
    x1 = q(1);
    x2 = q(2);
    x3 = q(3);
    x4 = q(4);
    y1 = q(5);
    y2 = q(6);
    y3 = q(7);
    y4 = q(8);
    z1 = q(9);
    z2 = q(10);
    z3 = q(11);
    z4 = q(12);

    x1y1 = r(1);
    x2y1 = r(2);
    x1y2 = r(3);
    x2y2 = r(4);
    x1z1 = r(5);
    x2z1 = r(6);
    x1z2 = r(7);
    x2z2 = r(8);
    y1z1 = r(9);
    y2z1 = r(10);
    y1z2 = r(11);
    y2z2 = r(12);

    x3y1 = s(1);
    x1y3 = s(2);
    x3z1 = s(3);
    x1z3 = s(4);
    y3z1 = s(5);
    y1z3 = s(6);

    x1y1z1 = t(1);
    x2y1z1 = t(2);
    x1y2z1 = t(3);
    x1y1z2 = t(4);


    % Calculando os coeficientes do sistema linear ------------------------
    K(1) = +1*x4-2*x2z2+1*z4;   % A2
    K(2) = +2*x2y2-2*x2z2-2*y2z2+2*z4;   % AB
    K(3) = +2*x3y1-2*x1y1z2;   % AD
    K(4) = +2*x3z1-2*x1z3;   % AE
    K(5) = +2*x2y1z1-2*y1z3;   % AF
    K(6) = +2*x3-2*x1z2;   % AG
    K(7) = +2*x2y1-2*y1z2;   % AH
    K(8) = +2*x2z1-2*z3;   % AI
    K(9) = +2*x2-2*z2;   % AJ
    K(10) = +1*y4-2*y2z2+1*z4;   % B2
    K(11) = +2*x1y3-2*x1y1z2;   % BD
    K(12) = +2*x1y2z1-2*x1z3;   % BE
    K(13) = +2*y3z1-2*y1z3;   % BF
    K(14) = +2*x1y2-2*x1z2;   % BG
    K(15) = +2*y3-2*y1z2;   % BH
    K(16) = +2*y2z1-2*z3;   % BI
    K(17) = +2*y2-2*z2;   % BJ
    K(18) = +1*x2y2;   % D2
    K(19) = +2*x2y1z1;   % DE
    K(20) = +2*x1y2z1;   % DF
    K(21) = +2*x2y1;   % DG
    K(22) = +2*x1y2;   % DH
    K(23) = +2*x1y1z1;   % DI
    K(24) = +2*x1y1;   % DJ
    K(25) = +1*x2z2;   % E2
    K(26) = +2*x1y1z2;   % EF
    K(27) = +2*x2z1;   % EG
    K(28) = +2*x1y1z1;   % EH
    K(29) = +2*x1z2;   % EI
    K(30) = +2*x1z1;   % EJ
    K(31) = +1*y2z2;   % F2
    K(32) = +2*x1y1z1;   % FG
    K(33) = +2*y2z1;   % FH
    K(34) = +2*y1z2;   % FI
    K(35) = +2*y1z1;   % FJ
    K(36) = +1*x2;   % G2
    K(37) = +2*x1y1;   % GH
    K(38) = +2*x1z1;   % GI
    K(39) = +2*x1;   % GJ
    K(40) = +1*y2;   % H2
    K(41) = +2*y1z1;   % HI
    K(42) = +2*y1;   % HJ
    K(43) = +1*z2;   % I2
    K(44) = +2*z1;   % IJ
    K(45) = num_pontos;   % J2


    % Montando o sistema linear Matriz*X = vetB ---------------------------
    Matriz = ...
    [2*K(1), K(2),  K(3),  K(4),  K(5),  K(6),  K(7),  K(8),  K(9); 
    K(2), 2*K(10), K(11), K(12), K(13), K(14), K(15), K(16), K(17); 
    K(3), K(11), 2*K(18), K(19), K(20), K(21), K(22), K(23), K(24); 
    K(4), K(12), K(19), 2*K(25), K(26), K(27), K(28), K(29), K(30); 
    K(5), K(13), K(20), K(26), 2*K(31), K(32), K(33), K(34), K(35);
    K(6), K(14), K(21), K(27), K(32), 2*K(36), K(37), K(38), K(39); 
    K(7), K(15), K(22), K(28), K(33), K(37), 2*K(40), K(41), K(42); 
    K(8), K(16), K(23), K(29), K(34), K(38), K(41), 2*K(43), K(44); 
    K(9), K(17), K(24), K(30), K(35), K(39), K(42), K(44), 2*K(45)];

    vetB = zeros(1,9);
    vetB(1) = +2*x2z2-2*z4;     % A
    vetB(2) = +2*y2z2-2*z4;     % B
    vetB(3) = +2*x1y1z2;        % D
    vetB(4) = +2*x1z3;          % E
    vetB(5) = +2*y1z3;          % F
    vetB(6) = +2*x1z2;          % G
    vetB(7) = +2*y1z2;          % H
    vetB(8) = +2*z3;            % I
    vetB(9) = +2*z2;            % J

    
    % Resolvendo o sistema linear -----------------------------------------
    vetB = -vetB;
    Coeficientes = inv(Matriz)*vetB';
    
    
    % Adicionando o coeficiente c no vetor de solução ---------------------
    a = Coeficientes(1);
    b = Coeficientes(2);
    c = 1 - a - b;
    d = Coeficientes(3);
    e = Coeficientes(4);
    f = Coeficientes(5);
    g = Coeficientes(6);
    h = Coeficientes(7);
    i = Coeficientes(8);
    j = Coeficientes(9);
    sol = [a; b; c; d; e; f; g; h; i; j];
    
    
    if (det(Matriz)==0)
        fprintf('Erro no método ElipsoidFitting (det = 0)')
    end
    

end