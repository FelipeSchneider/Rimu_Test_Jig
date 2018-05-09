function p = GeradorDeElipsoide(num_pontos, R, C, A)

    p = zeros(num_pontos, 3);

    Ax = A(1);
    Ay = A(2);
    Az = A(3);
    
    Rotx = [1,         0,        0;
            0,  cosd(Ax), sind(Ax); 
            0, -sind(Ax), cosd(Ax) ];
        
    Roty = [ cosd(Ay),   0, -sind(Ay);
                   0,    1,         0; 
             sind(Ay),   0,  cosd(Ay) ]; 

    Rotz = [ cosd(Az), sind(Az),   0; 
            -sind(Az), cosd(Az),   0;
                   0,         0,   1];
               
    Rot = Rotx * Roty * Rotz;
    
    for i =1:num_pontos   
        angle1 = 2*pi*rand();
        angle2 = pi*rand() - pi/2;
        p(i,1) = R(1)*cos(angle1)*cos(angle2);
        p(i,2) = R(2)*sin(angle1)*cos(angle2);
        p(i,3) = R(3)*sin(angle2);
        p(i,:) = p(i,:) * Rot';                   % Rotacao
        p(i, :) = p(i, :) + C;       % Translacao
    end

    
end