clear all;
%close all;

load('Aq/teste_2_1.mat');
quaternios_bno = Q;
USE_BNO_MAG = 1;
acqSize=time_sample*fs;
t = 0:1/fs:time_sample; %define o vetor de tempo
t(end) = [];            %elimina a ultima posição do vetor tempo

% %mag calibration
% DD2 = [x_mag_gaus' y_mag_gaus' z_mag_gaus'] * Ca' + repmat(Cb, acqSize, 1);
% x_mag_gaus = DD2(:,1)';
% z_mag_gaus = DD2(:,2)';
% y_mag_gaus = -DD2(:,3)';

%deve-se colocar os termos do BNO
% DD2 = [x_mag_bno_gaus' y_mag_bno_gaus' z_mag_bno_gaus'] * Ca' + repmat(Cb, acqSize, 1);
% x_mag_bno_gaus = DD2(:,1)';
% z_mag_bno_gaus = DD2(:,2)';
% y_mag_bno_gaus = -DD2(:,3)';

%Gyroscope statistics
Offset=[2.809936e+00,-5.056333e+00,-3.587206e+00]';
%var=[(1.934282e-01/180*pi)^2 (1.887641e-01/180*pi)^2 (4.747390e-01/180*pi)^2]';
var=[(1.934282e-01/180*pi) (1.887641e-01/180*pi) (4.747390e-01/180*pi)]';
%verificar esse quadrado

%Acquisition variables
GyroRate=zeros(3,acqSize);
Acc=zeros(3,acqSize);
Magn=zeros(3,acqSize);
Angles=zeros(3,acqSize);
AnglesObs=zeros(3,acqSize);
AccF=zeros(3,acqSize);
MagnF=zeros(3,acqSize);


qUpdate=zeros(4,acqSize);
%Initial quaternion values
qUpdate(:,1)=[1 0 0 0]';

%Observation vector
qOsserv=zeros(4,acqSize);
qOsserv(:,1)=[1 0 0 0]';

%----KALMAN MATRIXES
Q1=[var(1,1)+var(2,1)+var(3,1) -var(1,1)+var(2,1)-var(3,1) -var(1,1)-var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1)];
Q2=[-var(1,1)+var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1) -var(1,1)-var(2,1)+var(3,1)];
Q3=[-var(1,1)-var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1) -var(1,1)+var(2,1)-var(3,1)];
Q4=[var(1,1)-var(2,1)-var(3,1) -var(1,1)+var(2,1)-var(3,1) -var(1,1)+var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1)];
Qmatrix=[Q1;Q2;Q3;Q4];

H=eye(4,4);
R=eye(4,4)*1;

qPredicted=zeros(4,acqSize);
qPredicted(:,1)=ones(4,1)*0.5;
P_Update=eye(4,4)*1;
%----------

i=1;
dt=1/fs;

[bAcc,aAcc] = butter(3,0.25,'low');
[bMagn,aMagn] = butter(2,0.25,'low');

magnF_Length=20;
accF_Length=20;

%Bring up the filters
while(i<=accF_Length+4)
    Acc(:,i)=acc_imu_g(:,i);
    if(USE_BNO_MAG==0)
        Magn(:,i)=mag_imu_gaus(:,i);
    else
        Magn(:,i)=mag_bno_gaus(:,i);
    end
    GyroRate(:,i)=((giro_imu_dps(:,i)-Offset(:,1))/180)*pi;
    
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
    qPredicted(:,i)=[0.5 0.5 0.5 0.5]';
    qUpdate(:,i)=[0.5 0.5 0.5 0.5]';
    qOsserv(:,i)=[0.5 0.5 0.5 0.5]';
end

while(i<=acqSize)
    Acc(:,i)=acc_imu_g(:,i);
    if(USE_BNO_MAG==0)
        Magn(:,i)=mag_imu_gaus(:,i);
    else
        Magn(:,i)=mag_bno_gaus(:,i);
    end
    GyroRate(:,i)=((giro_imu_dps(:,i)-Offset(:,1))/180)*pi;
    
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
    Angles(:,i)=GetAnglesFromQuaternion(qUpdate(:,i));
    AnglesObs(:,i)=GetAnglesFromQuaternion(qOsserv(:,i));
    
    
    %END KALMAN FILTERING
    i=i+1;
end
%figure;
%    subplot(3,1,1);plot(t,Acc(1,:),'b',t,AccF(1,:),'r',t,Magn(1,:),'g',t,MagnF(1,:),'c');legend('Acc X','Acc X Filtered','Magn X','Magn X Filtered');grid;xlabel('time (sec)');ylabel('angle (deg)');
%    subplot(3,1,2);plot(t,Acc(2,:),'b',t,AccF(2,:),'r',t,Magn(2,:),'g',t,MagnF(2,:),'c');legend('Acc Y','Acc Y Filtered','Magn Y','Magn Y Filtered');grid;xlabel('time (sec)');ylabel('angle (deg)');
%    subplot(3,1,3);plot(t,Acc(3,:),'b',t,AccF(3,:),'r',t,Magn(3,:),'g',t,MagnF(3,:),'c');legend('Acc Z','Acc Z Filtered','Magn Z','Magn Z Fitlered');grid;xlabel('time (sec)');ylabel('angle (deg)');


% figure(1);
%     subplot(4,1,1);plot(t,qOsserv(1,1:acqSize));grid;legend('q0 Observed');
%     subplot(4,1,2);plot(t,qOsserv(2,1:acqSize));grid;legend('q1 Observed');
%     subplot(4,1,3);plot(t,qOsserv(3,1:acqSize));grid;legend('q2 Observed');
%     subplot(4,1,4);plot(t,qOsserv(4,1:acqSize));grid;legend('q3 Observed');
% 
% figure(2);
%     subplot(4,1,1);plot(t,qUpdate(1,1:acqSize));grid;legend('q0 Estimated');
%     subplot(4,1,2);plot(t,qUpdate(2,1:acqSize));grid;legend('q1 Estimated');
%     subplot(4,1,3);plot(t,qUpdate(3,1:acqSize));grid;legend('q2 Estimated');
%     subplot(4,1,4);plot(t,qUpdate(4,1:acqSize));grid;legend('q3 Estimated');
    
%figure;
%    subplot(3,1,1);plot(t,Angles(1,1:acqSize));grid;legend('Roll');
%   subplot(3,1,2);plot(t,Angles(2,1:acqSize));grid;legend('Pitch');
%   subplot(3,1,3);plot(t,Angles(3,1:acqSize));grid;legend('Yaw');

   %figure;
    %subplot(3,1,1);plot(t,AnglesObs(1,1:acqSize));grid;legend('Roll');
   %subplot(3,1,2);plot(t,AnglesObs(2,1:acqSize));grid;legend('Pitch');
   %subplot(3,1,3);plot(t,AnglesObs(3,1:acqSize));grid;legend('Yaw');

%INEMO_Disconnection(handle_dev);


% figure(3);
% plot(t,quaternios_bno/2^14'); grid on;
% title('Quaternios'); legend('w','x','y','z'); hold all;
% plot(t,qOsserv(1,1:acqSize),'--','Color',[0,0.4470,0.7410]);
% plot(t,qOsserv(2,1:acqSize),'--','Color',[0.8500, 0.3250, 0.0980]);
% plot(t,qOsserv(3,1:acqSize),'--','Color',[0.9290, 0.6940, 0.125]);
% plot(t,qOsserv(4,1:acqSize),'--','Color',[0.4940, 0.1840, 0.5560]);


[r1_imu r2_imu r3_imu] = quat2angle(qUpdate');
[r1_bno r2_bno r3_bno] = quat2angle(quaternios_bno');

figure(4);hold all;
subplot(311); plot(t,[r1_bno r1_imu]'); grid on;
title('Angulos de Euler'); hold all; legend('BNO','IMU');ylabel('Yaw');
subplot(312); plot(t,[r2_bno r2_imu]'); grid on;
hold all; legend('BNO','IMU');ylabel('Pitch');
subplot(313); plot(t,[r3_bno r3_imu]'); grid on;
hold all; legend('BNO','IMU');ylabel('Roll');
