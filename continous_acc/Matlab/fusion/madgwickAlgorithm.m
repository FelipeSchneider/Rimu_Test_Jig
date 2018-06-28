function [ quaternion ] = madgwickAlgorithm(Acc, GyroRate, Magn, fs, ...
                            gyro_off, Beta, init_Q)
%[ quaternion ] = madgwickAlgorithm(Acc, GyroRate, Magn, fs, ...
%                            gyro_off, Beta)
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
%   Beta        Madgwick's gain
%   init_Q      Initial quaternion orientation
if(nargin == 7)
    AHRS = MadgwickAHRS('SamplePeriod', 1/fs, 'Beta', Beta, 'Quaternion', init_Q);
else
    AHRS = MadgwickAHRS('SamplePeriod', 1/fs, 'Beta', Beta);
end
for i=1:length(Acc)
    Acc(:,i)=Acc(:,i)/norm(Acc(:,i));
    Magn(:,i)=Magn(:,i)/norm(Magn(:,i));
end
quaternion = zeros(length(Acc), 4);
GyroRate = GyroRate - gyro_off;
GyroRate = GyroRate'*(pi/180);
Acc = Acc'; Magn = Magn';
for t = 1:length(Acc)
    AHRS.Update(GyroRate(t,:), Acc(t,:), Magn(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

end

