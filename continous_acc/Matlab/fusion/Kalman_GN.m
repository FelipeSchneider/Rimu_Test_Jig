function [ qUpdate ] = Kalman_GN(Acc, GyroRate, Magn, fs, gyro_off, gyro_var,...
                            sigmaR, P_guess, acc_lpc, mag_lpc, quat_init)
%[ quaternion ] = Kalman_GN(Acc, GyroRate, Magn, fs, gyro_off, gyro_var,...
%                            sigmaR, P_guess, acc_lpc, mag_lpc, quat_init)
%Kalman_GDNhe fusion of data based on the Gauss-Newton version of the filter
%provided by the iNEMO group
%   Acc         accelerometer readings. The input format must be
%               3xNreadings. The units are not considered. The measure will
%               be normalized.
%   GyroRate    gyroscope readings. The input format must be 3xNreadings
%               and the units must be degrees per second.
%   Magn        magnetometer readings. The input format must be
%               3xNreadings.The units are not considered. The measure will
%               be normalized.
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

acqSize = length(Acc);
AccF=zeros(3,acqSize);
MagnF=zeros(3,acqSize);

qUpdate=zeros(4,acqSize);
qOsserv=zeros(4,acqSize);
qPredicted=zeros(4,acqSize);
if ~exist('quat_init','var')
    qUpdate(:,1)=[1 0 0 0]';
    qOsserv(:,1)=[1 0 0 0]';
    qPredicted(:,1)=[1 0 0 0]';
else
    qUpdate(:,1) = quat_init;
    qOsserv(:,1) = quat_init;
    qPredicted(:,1) = quat_init;
end

gyro_var = gyro_var*pi/180;


%----KALMAN MATRIXES
Q1=[gyro_var(1,1)+gyro_var(2,1)+gyro_var(3,1) -gyro_var(1,1)+gyro_var(2,1)-gyro_var(3,1) -gyro_var(1,1)-gyro_var(2,1)+gyro_var(3,1) gyro_var(1,1)-gyro_var(2,1)-gyro_var(3,1)];
Q2=[-gyro_var(1,1)+gyro_var(2,1)-gyro_var(3,1) gyro_var(1,1)+gyro_var(2,1)+gyro_var(3,1) gyro_var(1,1)-gyro_var(2,1)-gyro_var(3,1) -gyro_var(1,1)-gyro_var(2,1)+gyro_var(3,1)];
Q3=[-gyro_var(1,1)-gyro_var(2,1)+gyro_var(3,1) gyro_var(1,1)-gyro_var(2,1)-gyro_var(3,1) gyro_var(1,1)+gyro_var(2,1)+gyro_var(3,1) -gyro_var(1,1)+gyro_var(2,1)-gyro_var(3,1)];
Q4=[gyro_var(1,1)-gyro_var(2,1)-gyro_var(3,1) -gyro_var(1,1)+gyro_var(2,1)-gyro_var(3,1) -gyro_var(1,1)+gyro_var(2,1)-gyro_var(3,1) gyro_var(1,1)+gyro_var(2,1)+gyro_var(3,1)];
Qmatrix=[Q1;Q2;Q3;Q4];

H=eye(4,4);
R=eye(4,4)*sigmaR;
P_Update=eye(4,4)*P_guess;
%----------
i=1;
dt=1/fs;

[bAcc,aAcc] = butter(4,acc_lpc,'low');
[bMagn,aMagn] = butter(2,mag_lpc,'low');

magnF_Length=20;
accF_Length=20;

%Bring up the filters
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
    if ~exist('quat_init','var')
        qPredicted(:,i)=[1 0 0 0]';
        qUpdate(:,i)=[1 0 0 0]';
        qOsserv(:,i)=[1 0 0 0]';
    else
        qUpdate(:,i) = quat_init;
        qPredicted(:,i) = quat_init;
        qOsserv(:,i) = quat_init;
    end
end

while(i<=acqSize)        
    GyroRate(:,i)=((GyroRate(:,i)-gyro_off(:,1))/180)*pi;
    
%     GyroRate(1,i)=(GyroRate(1,i)+GyroRate(1,i-1))/2;
%     GyroRate(2,i)=(GyroRate(2,i)+GyroRate(2,i-1))/2;
%     GyroRate(3,i)=(GyroRate(3,i)+GyroRate(3,i-1))/2;
    
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
    qOsserv(:,i)=GaussNewtonMethod(qOsserv(:,i-1),AccF(:,i),MagnF(:,i));
    qOsserv(:,i)=qOsserv(:,i)/norm(qOsserv(:,i));
    %END OSSERVATION COMPUTING
    
    %KALMAN FILTERING
    const=dt/2;
    %F matrix computing
    F1=[1 -const*GyroRate(1,i) -const*GyroRate(2,i) -const*GyroRate(3,i)];
    F2=[const*GyroRate(1,i) 1 const*GyroRate(3,i) -const*GyroRate(2,i)];
    F3=[const*GyroRate(2,i) -const*GyroRate(3,i) 1 const*GyroRate(1,i)];
    F4=[-const*GyroRate(3,i) const*GyroRate(2,i) -const*GyroRate(1,i) 1];
    
    F=[F1;F2;F3;F4];
    qPredicted(:,i)=F*qUpdate(:,i-1);
    qPredicted(:,i)=qPredicted(:,i)/norm(qPredicted(:,i));
    Q=Qmatrix;
    
    P_Predicted=F*P_Update*F'+Q;
    
    K=P_Predicted*H'*(H*P_Predicted*H'+R)^-1;
    qUpdate(:,i)=qPredicted(:,i)+K*(qOsserv(:,i)-H*qPredicted(:,i));
    qUpdate(:,i)=qUpdate(:,i)/norm(qUpdate(:,i));
    
    P_Update=(eye(4,4)-K*H)*P_Predicted;
%   Angles(:,i)=GetAnglesFromQuaternion(qUpdate(:,i));
    
    %END KALMAN FILTERING
    i=i+1;
end

end

