function [ quaternion ] = KalmanV13( Acc, GyroRate, Magn, fs, gyro_off, gyro_var,...
                            sigmaR, P_guess, acc_lpc, mag_lpc, quat_init)
%KalmanV13 make the fusion of data based on the 13th version of the filter
%provided by the iNEMO group
%   Acc         accelerometer readings. The input format must be 3xNreadings
%   GyroRate    gyroscope readings. The input format must be 3xNreadings
%   Magn        magnetometer readings. The input format must be 3xNreadings
%   fs          sampling frequency in Hz
%   gyro_off    Gyroscope offset in dps. The input format must be 3x1
%   gyro_var    Gyroscope variance in dps. The input format must be 3x1
%   sigmaR      This is a kalman filter parameter. The input format must be 4x1
%   P_guess     Initial guess of the autocovariance matrix.
%   acc_lpc     Low pass cutoff of the 4th order butterwolf filter that will be
%               applied to the accelerometer readings. The value is a
%               percentage of the sampling rate 
%   mag_lpc     Low pass cutoff of the 3rd order butterwolf filter that will be
%               applied to the magnetometer readings. The value is a
%               percentage of the sampling rate 
%   quat_init   quaternion initial position. The input format must be 4x1
addpath('Kalman_iNemo\v13');      % include quaternion library
acqSize = length(Acc);

%Acquisition variables
% GyroRate=zeros(3,acqSize);
% Acc=zeros(3,acqSize);
% Magn=zeros(3,acqSize);
% Angles=zeros(3,acqSize);
% AnglesObs=zeros(3,acqSize);
AccF=zeros(3,acqSize);
MagnF=zeros(3,acqSize);

qUpdate=zeros(4,acqSize);
if ~exist('arg1','var')
    qUpdate(:,1)=[1 0 0 0]';
else
    qUpdate(:,1) = quat_init;
end

Q1=[gyro_var(1,1)+gyro_var(2,1)+gyro_var(3,1) -gyro_var(1,1)+gyro_var(2,1)-gyro_var(3,1) -gyro_var(1,1)-gyro_var(2,1)+gyro_var(3,1) gyro_var(1,1)-gyro_var(2,1)-gyro_var(3,1)];
Q2=[-gyro_var(1,1)+gyro_var(2,1)-gyro_var(3,1) gyro_var(1,1)+gyro_var(2,1)+gyro_var(3,1) gyro_var(1,1)-gyro_var(2,1)-gyro_var(3,1) -gyro_var(1,1)-gyro_var(2,1)+gyro_var(3,1)];
Q3=[-gyro_var(1,1)-gyro_var(2,1)+gyro_var(3,1) gyro_var(1,1)-gyro_var(2,1)-gyro_var(3,1) gyro_var(1,1)+gyro_var(2,1)+gyro_var(3,1) -gyro_var(1,1)+gyro_var(2,1)-gyro_var(3,1)];
Q4=[gyro_var(1,1)-gyro_var(2,1)-gyro_var(3,1) -gyro_var(1,1)+gyro_var(2,1)-gyro_var(3,1) -gyro_var(1,1)+gyro_var(2,1)-gyro_var(3,1) gyro_var(1,1)+gyro_var(2,1)+gyro_var(3,1)];
Qmatrix=[Q1;Q2;Q3;Q4];

H=eye(4,4);

R=[sigmaR(1,1) 0 0 0;0 sigmaR(2,1) 0 0;0 0 sigmaR(3,1) 0;0 0 0 sigmaR(4,1)];

qPredicted=zeros(4,acqSize);
qPredicted(:,1)=[0.05 0.05 0.05 0.05]';
P_Update=eye(4,4)*P_guess;

i=1;
dt=1/fs;

[bAcc,aAcc] = butter(4,acc_lpc,'low');
[bMagn,aMagn] = butter(2,mag_lpc,'low');

magnF_Length=20;
accF_Length=20;

while(i<=accF_Length+4)
    GyroRate(:,i)=((GyroRate(:,i)-gyro_off(:,1))/180)*pi;
    
    Acc(:,i)=Acc(:,i)/norm(Acc(:,i));
    Magn(:,i)=Magn(:,i)/norm(Magn(:,i));
    if(i<=accF_Length)
        AccF(:,i)=MyFilter(bAcc,aAcc,Acc(:,:));
    else
        AccF(:,i)=MyFilter(bAcc,aAcc,Acc(:,i-accF_Length:i));
    end
    if(i<=magnF_Length)
        MagnF(:,i)=MyFilter(bMagn,aMagn,Magn(:,:));
    else
        MagnF(:,i)=MyFilter(bMagn,aMagn,Magn(:,i-magnF_Length:i));
    end
    MagnF(:,i)=MagnF(:,i)/norm(MagnF(:,i));
    AccF(:,i)=AccF(:,i)/norm(AccF(:,i));
    i=i+1;
    if ~exist('arg1','var')
        qPredicted(:,i)=[0.5 0.5 0.5 0.5]';
        qUpdate(:,i)=[0.5 0.5 0.5 0.5]';
        qOsserv(:,i)=[0.5 0.5 0.5 0.5]';
    else
        qUpdate(:,1) = quat_init;
        qPredicted(:,1) = quat_init;
        qOsserv(:,1) = quat_init;
    end
end

while(i<=acqSize)
    GyroRate(:,i)=((GyroRate(:,i)-gyro_off(:,1))/180)*pi;
    
    %Normalization and filtering
    Acc(:,i)=Acc(:,i)/norm(Acc(:,i));
    Magn(:,i)=Magn(:,i)/norm(Magn(:,i));
    
    AccF(:,i)=MyFilter(bAcc,aAcc,Acc(:,i-accF_Length:i));
    MagnF(:,i)=MyFilter(bMagn,aMagn,Magn(:,i-magnF_Length:i));
    
    MagnF(:,i)=MagnF(:,i)/norm(MagnF(:,i));
    AccF(:,i)=AccF(:,i)/norm(AccF(:,i));
    %----End Acquisition
    
    %OBSERVATION COMPUTING
    %Gauss Newton step 
    %qOss=GaussNewtonMethod(qUpdate(2,i-1),qUpdate(3,i-1),qUpdate(4,i-1),qUpdate(1,i-1),AccF(:,i),MagnF(:,i),MagnF(:,17));
    %qOsserv(:,i)=[qOss(4,1); qOss(2:4,1)];
    
    %Gradient Descent
    dq=0.5*(QuaternionProduct(qUpdate(:,i-1),[0 GyroRate(1,i) GyroRate(2,i) GyroRate(3,i)]'));
    %mu=norm(dq)*0.6*(1/(norm(dq)^(71/100)));
    mu=10*norm(dq)*dt;
    qOsserv(:,i)=GradientDescent(AccF(:,i),MagnF(:,i),qOsserv(:,i-1),mu);
    
    qOsserv(:,i)=qOsserv(:,i)/norm(qOsserv(:,i));
    %END OSSERVATION COMPUTING
    
    %KALMAN FILTERING
    
    %F matrix computing
    F1=[1 -dt/2*GyroRate(1,i) -dt/2*GyroRate(2,i) -dt/2*GyroRate(3,i)];
    F2=[dt/2*GyroRate(1,i) 1 dt/2*GyroRate(3,i) -dt/2*GyroRate(2,i)];
    F3=[dt/2*GyroRate(2,i) -dt/2*GyroRate(3,i) 1 dt/2*GyroRate(1,i)];
    F4=[-dt/2*GyroRate(3,i) dt/2*GyroRate(2,i) -dt/2*GyroRate(1,i) 1];
    
    F=[F1;F2;F3;F4];
    qPredicted(:,i)=F*qUpdate(:,i-1);
    
    Q=Qmatrix;
    
    P_Predicted=F*P_Update*F'+Q;
    
    K=P_Predicted*H'*(H*P_Predicted*H'+R)^-1;
    
    qUpdate(:,i)=qPredicted(:,i)+K*(qOsserv(:,i)-H*qPredicted(:,i));
    qUpdate(:,i)=qUpdate(:,i)/norm(qUpdate(:,i));
    
    P_Update=(eye(4,4)-K*H)*P_Predicted;
%     Angles(:,i)=GetAnglesFromQuaternion(qUpdate(:,i));
%     AnglesObs(:,i)=GetAnglesFromQuaternion(qOsserv(:,i));
    %END KALMAN FILTERING
    i=i+1;
end
quaternion = qUpdate;
end

