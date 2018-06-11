function [W,fGA] = updateX(X,alpha,q_jig, Xmin, Xmax ,acc_data, gyro_data, mag_data, bw, fn, mn, fs)
%   X           - group of particles [nParameters, nParticles]
%   alpha       - crossover expantion factor
%   q_jig       - response of real jig orientation
%   Xmin        - Lower limit of search [1, nParameters]
%   Xmax        - Higher limit of search [1, nParameters]
%   acc_data    - linear acceleration data [3,N]
%   gyro_data   - angular speed in degrees per second [3,N]
%   mag_data    - magnetic data [3,N]
%   bw          - Gyroscope bias 
%   fn          - Gravity vector in navigation frame [3x1]
%   mn          - Magetic field vector in navigation frame [3x1]
%   fs          - sampling frequency
% Returns:
%   W           - New set of particles
%   fGA         - Fit of the new set of particles
    
    c = size(X,2);
    parameters = size(X,1);
    W = zeros(size(X));
    fGA = zeros(c,1);
            syms q
    for k = 1:c
        P1 = X(:,round(length(X)*rand(1) + 0.5));
        P2 = X(:,round(length(X)*rand(1) + 0.5));
        Beta = (2*alpha+1)*rand(1)-alpha;
        C1 = P1 + Beta*(P2-P1); 
        C2 = P2 + Beta*(P1-P2);

        %assure that the limits are respected:
        for j=1:parameters
           if(C1(j) < Xmin(j))
              C1(j) = Xmin(j);
           end
           if(C2(j) < Xmin(j))
              C2(j) = Xmin(j);
           end
           if(C1(j) > Xmax(j))
              C1(j) = Xmax(j);
           end
           if(C2(j) > Xmax(j))
              C2(j) = Xmax(j);
           end
        end

    %     Y1_out = Sys_response(C1,sys_input,t);
    %     Y1 = Sys_fit(Y1_out,y_ideal);
    %     
    %     Y2_out = Sys_response(C2,sys_input,t);
    %     Y2 = Sys_fit(Y2_out,y_ideal);
        [q_out] = Kalman_response(C1,acc_data, gyro_data, mag_data, bw, fn, mn, fs, Xmin, Xmax);
        [fit_1] = Kalman_fit(q_out,q_jig);

        [q_out] = Kalman_response(C2,acc_data, gyro_data, mag_data, bw, fn, mn, fs, Xmin, Xmax);
        [fit_2] = Kalman_fit(q_out,q_jig);

        if fit_1 <= fit_2
            W(:,k) = C1;
            fGA(k,1) = fit_1;
        else
            W(:,k) = C2;
            fGA(k,1) = fit_2;
        end
    end
end