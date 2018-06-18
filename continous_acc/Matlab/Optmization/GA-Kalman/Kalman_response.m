function [q_out, X] = Kalman_response(X,acc_data, gyro_data, mag_data, bw, fn, mn, fs, Xmin, Xmax)
%Kalman_response returns the response (in quaternions) of the kalman filter for each group
%of parameters X 
%   X   - group of particles [nParameters, nParticles]
%   acc_data    - linear acceleration data [3,N]
%   gyro_data   - angular speed in degrees per second [3,N]
%   mag_data    - magnetic data [3,N]
%   bw  - Gyroscope bias [3,1]
%   fn  - Gravity vector in navigation frame [3x1]
%   mn  - Magetic field vector in navigation frame [3x1]
%   fs  - sampling frequency
%   Xmin - Lower limit of search [1, nParameters]
%   Xmax - Higher limit of search [1, nParameters]

 c = size(X,2);                         %number of particles
 q_out = zeros(length(acc_data),4,c);   %first position the time stamp
                                        %second position define the quaternion
                                        %last position define the particle
n_parameters = length(Xmin);
%R is the measurement noise covariance matrix
%P is the error covariance matrix
%warning('off', 'MATLAB:illConditionedMatrix');
warning('off');
for j=1:c
%    fprintf('Kalman response, particle n: %d \r',j)
    R(1:3,1:3)=diag([X(1,j), X(1,j), X(1,j)]);        %Initial Covariance matrix
    R(4:6,4:6)=diag([X(2,j), X(2,j), X(2,j)]);
    P(1:3,1:3)=diag([X(3,j), X(3,j), X(3,j)]);        %Initial Covariance matrix
    P(4:6,4:6)=diag([X(4,j), X(4,j), X(4,j)]);
    [q, ~, ~] = Gyro_lib_quat(P, bw*(pi/180), gyro_data*(pi/180*(1/fs)), X(5,j), X(6,j),...
    acc_data, mag_data, fn, mn, 1/fs, R);  
    
    while(isnan(q(30,:))) % if the kalman is not well scalled (error during the fusion process)
        %q(9,:) = [1 0 0 0];
%        fprintf('Redo Kalman response for particle number %d \r',j)
        for k=1:n_parameters
            X(k,j) = Xmin(k) + (Xmax(k)-Xmin(k))*rand(1);
        end
        
        R(1:3,1:3)=diag([X(1,j), X(1,j), X(1,j)]);        %Initial Covariance matrix
        R(4:6,4:6)=diag([X(2,j), X(2,j), X(2,j)]);
        P(1:3,1:3)=diag([X(3,j), X(3,j), X(3,j)]);        %Initial Covariance matrix
        P(4:6,4:6)=diag([X(4,j), X(4,j), X(4,j)]);
        [q, ~, ~] = Gyro_lib_quat(P, bw*(pi/180), gyro_data*(pi/180*(1/fs)), X(5,j), X(6,j),...
        acc_data, mag_data, fn, mn, 1/fs, R);
    end
    
    q(isnan(q(:,1)),1) = 1; %replace NaN by unit vector, usually the last measurement is NaN
    q(isnan(q(:,2)),2:4) = 0;
    
    q = quatconj(q); %the kalman gyrolib returns the quaternion conjugate
    
    %correct the initial position
    q_zero = avg_quaternion_markley(q(380:480,:));
    q = quatmultiply(quatconj(q_zero'),q);
    q_out(:,:,j) = q;
end
warning('on');
%warning('on', 'MATLAB:illConditionedMatrix');