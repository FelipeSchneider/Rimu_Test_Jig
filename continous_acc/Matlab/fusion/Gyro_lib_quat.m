function [q, P, bw_] = Gyro_lib_quat(P, bw, dwb, ng, ngb, Fb, Mb, fn, mn, dt, R)
% [q, P, bw] = ahrs_quat(q, P, bw, dwb, fb, mb, fn, mn, dt)
% Implements the quaternion AHRS algorithm usin measurements
% from the three-axis gyroscope and accelerometer,
%  vector magnetometer
%
%   Input arguments:
%   P   - Kalman filter covarince matrix [6x6]
%   bw  - Gyroscopes biases vector [3x1]
%   dwb - Intergral of angular rate vector over the computer cycle [3xN]
%   ng  - Gyro errors noise, in the original code the vaule of 1e-4 is used
%   ngb - Gyro bias noise, original value 1e-7
%   Fb  - Acceleration vector in body frame [3xN]
%   Mb  - Magnetic field vector in body frame [3xN]
%   fn  - Gravity vector in navigation frame [3x1]
%   mn  - Magetic field vector in navigation frame [3x1]
%   dt  - Computer cycle, sec.
%   R   - Measurement noise covariance matrix, original value diag([1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1])
%
%   Output arguments:
%   q  - Updated attitude quaternion [1x4]
%   P  - Updated Kalman filter covarince matrix [6x6]
%   bw - Updated gyroscopes biases vector [3x1]
Nsim = length(dwb);
q = zeros(Nsim,4);
q(1,:) = [1 0 0 0];
bw_ = zeros(Nsim,3);
for i=2:Nsim
%     fb  = Fb(i,:)'./norm(Fb(i,:));
%     mb  = Mb(i,:)'./norm(Mb(i,:));
    fb  = Fb(:,i)./norm(Fb(:,i));
    mb  = Mb(:,i)./norm(Mb(:,i));
    % Correct attitide increments  with estimated biases
    dwb_now  = dwb(:,i)-bw;

    % Attitude Quaternion Mechanization
    %Quaternion from k+1 body axes to k body axes
    gamma = norm(dwb_now);
    if (gamma~=0)
        lambda = [cos(gamma/2), dwb_now(1)*sin(gamma/2)/gamma, dwb_now(2)*sin(gamma/2)/gamma,...
            dwb_now(3)*sin(gamma/2)/gamma];
    else
        lambda = [1, 0, 0, 0];
    end
    %Update q for body motion using gyro measurements only
    q(i,:) = quat_mult(quat_conj(lambda),q(i-1,:));

    %Normalize quaternion
    q(i,:) = q(i,:)/sqrt(q(i,:)*q(i,:)');

    %% Kalman predict
    %System dynamics matrix F and system noise covariance matrix Q
    %Continuous-time system matrix
    A = zeros(6,6);
    A(1:3,4:6) = quat_dcm(q);
    %State transition matrix
    F = eye(6)+A*dt+A*A*dt*dt/2;
    %Noise-input mapping matrix
    G = eye(6);

    %Gyro errors noise
    %ng = 1e-4;
    %Gyro bias noise
    %ngb = 1e-7;
    %System noise
    Qn = diag([ng, ng, ng, ngb, ngb, ngb]);

    %Trapezioidal integration
    Q = 1/2*(F*G*Qn*G'+G*Qn*G'*F')*dt;

    %Covariance predict
    P = F*P*F'+Q;

    %% Measurements
    %Estimated measurements:
    %"Gravity" vector estimate in navigation frame
    fn_hat = quat_rot(q(i,:),fb);
    %"Magnetic field" vector estimate in navigation frame
    mn_hat = quat_rot(q(i,:),mb);

    %Measurement vector
    v = zeros(6,1);
    v(1:3,1) = fn-fn_hat;
    v(4:6,1) = mn-mn_hat;

    %Measurement matrix
    H = zeros(6,6);
    H(1:3,1:3) = skew(fn_hat);
    H(4:6,1:3) = skew(mn_hat);

%     %Measurement noise covariance matrix
%     R = diag([1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1]);

    %% Kalman Update
    I = eye(6);
    S = H*P*H+R;
    K = (P*H')/S;
    P = (I-K*H)*P*(I-K*H)'+K*R*K';
    x = K*v;

    %% Correct Attitude Quaternion
    %Error quaternion
    qe = [1, x(1)/2, x(2)/2, x(3)/2];
    qe = qe/sqrt(qe*qe');
    %Correct esimated attitude quaternion
    q(i,:) = quat_mult(q(i,:), qe);

    %% Update gyro bias estimate
    bw = bw+x(4:6,1);
    bw_(i,:) = bw*dt;
end

end


