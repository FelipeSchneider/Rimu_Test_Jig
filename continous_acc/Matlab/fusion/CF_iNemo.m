function [ qFilt ] = CF_iNemo(Acc, GyroRate, Magn, fs, gyro_off,...
                            acc_lpc, mag_lpc, filt_coef, quat_init)
%[ qFilt ] = CF_iNemo(Acc, GyroRate, Magn, fs, gyro_off,...
%                            acc_lpc, mag_lpc, filt_coef, quat_init)
%Execute the Complementary filter provided by the iNemo Group
%   Acc         accelerometer readings. The input format must be
%               3xNreadings. The units are not considered. The measure will
%               be normalized.
%   GyroRate    gyroscope readings. The input format must be 3xNreadings
%               and the units must be degrees per second
%   Magn        magnetometer readings. The input format must be
%               3xNreadings.The units are not considered. The measure will
%               be normalized.
%   fs          sampling frequency in Hz
%   gyro_off    Gyroscope offset in dps. The input format must be 3x1
%   acc_lpc     Low pass cutoff of the 4th order butterwolf filter that will be
%               applied to the accelerometer readings. The value is a
%               percentage of the sampling rate 
%   mag_lpc     Low pass cutoff of the 3rd order butterwolf filter that will be
%               applied to the magnetometer readings. The value is a
%               percentage of the sampling rate 
%   filt_coef   First order low pass IRR coeficient that will be applied at the 
%               output quaternions.
%   quat_init   quaternion initial position. The input format must be 4x1

if(filt_coef>1)
    error('Invalid filt_coef argument, it must be between 0 and 1');
elseif(filt_coef<0)
   error('Invalid filt_coef argument, it must be between 0 and 1');
end

dt = 1/fs;
acqSize = length(Acc);
AccF=zeros(3,acqSize);
MagnF=zeros(3,acqSize);
mu=zeros(1,acqSize);
dqnorm=zeros(1,acqSize);
dq=zeros(4,acqSize);


qOsserv=zeros(4,acqSize);   %Observation vector (accelerometer and magnetometer)
qGyroFilt=zeros(4,acqSize); %Gyrofilt vector
qFilt=zeros(4,acqSize);     %Filtered vector

if ~exist('quat_init','var')
        qGyroFilt(:,1)  = [1 0 0 0]';
        qFilt(:,1)      = [1 0 0 0]';
        qOsserv(:,1)    = [1 0 0 0]';
else
        qGyroFilt(:,1)  = quat_init;
        qFilt(:,1)      = quat_init;
        qOsserv(:,1)    = quat_init;
end

[bAcc,aAcc] = butter(4,acc_lpc,'low');
[bMagn,aMagn] = butter(2,mag_lpc,'low');
magnF_Length=20;
accF_Length=20;
i = 1;
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
    qOsserv(:,i)=qOsserv(:,i-1);
    qFilt(:,i)=qFilt(:,i-1);
    qGyroFilt(:,i)=qGyroFilt(:,i-1);
end

while(i<=acqSize)        
    GyroRate(:,i)=((GyroRate(:,i)-gyro_off(:,1))/180)*pi;
    
    GyroRate(1,i)=(GyroRate(1,i)+GyroRate(1,i-1))/2;
    GyroRate(2,i)=(GyroRate(2,i)+GyroRate(2,i-1))/2;
    GyroRate(3,i)=(GyroRate(3,i)+GyroRate(3,i-1))/2;
    
    AccF(:,i)=MyFilter(bAcc,aAcc,Acc(:,i-accF_Length:i));
    MagnF(:,i)=MyFilter(bMagn,aMagn,Magn(:,i-magnF_Length:i));
    
    MagnF(:,i)=MagnF(:,i)/norm(MagnF(:,i));
    AccF(:,i)=AccF(:,i)/norm(AccF(:,i));
    %----End Acquisition
    
    dq(:,i)=0.5*(QuaternionProduct(qFilt(:,i-1),[0 GyroRate(1,i) GyroRate(2,i) GyroRate(3,i)]'));
    dqnorm(1,i)=norm(dq(:,i));
    mu(1,i)=10*dqnorm(1,i)*dt;
    qOsserv(:,i)=GradientDescent(AccF(:,i),MagnF(:,i),qOsserv(:,i-1),mu(1,i));
    qOsserv(:,i)=qOsserv(:,i)/norm(qOsserv(:,i));
    
    if(i<=accF_Length+10)
        qGyroFilt(:,i)=qOsserv(:,i);
        qFilt(:,i)=qOsserv(:,i);
    else
        qGyroFilt(:,i)=qFilt(:,i-1)+dq(:,i)*dt;
        qGyroFilt(:,i)=qGyroFilt(:,i)/norm(qGyroFilt(:,i));

        dqnorm(1,i)=norm(dq(:,i));
        mu(1,i)=10*dqnorm(1,i)*dt;
        qOsserv(:,i)=GradientDescent(AccF(:,i),MagnF(:,i),qOsserv(:,i-1),mu(1,i));
        qOsserv(:,i)=qOsserv(:,i)/norm(qOsserv(:,i));

        qFilt(:,i)=qGyroFilt(:,i)*filt_coef + qOsserv(:,i)*(1-filt_coef); 
        qFilt(:,i)=qFilt(:,i)/norm(qFilt(:,i));
        
%         qFilt_aux = wavg_quaternion_markley([qGyroFilt(:,i)'; qOsserv(:,i)'], [filt_coef, 1-filt_coef]);
%         qFilt(:,i)=(qFilt_aux/norm(qFilt_aux))';
    end
    i=i+1;
    
end

end

