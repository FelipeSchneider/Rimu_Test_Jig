function [ t_match, base_match, top_match ] = matchJigIMU(t_jig, t_imu, b_angle, t_angle )
%matchJigIMU match with a selective resample the time and angle vectors of
%            time and angle for the jig in function of the IMU time
%t__jig      Jig time vector
%t_imu       Imu time vector
%b_angle     Jig base angle
%t_angle     Jig top angle
%
%t_match     Reconstruction of the jig time as function of imu time vector
%b_match     Reconstruction of the jig base angle
%t_match     Reconstruction of the jig top angle

t_match = zeros(size(t_imu));
base_match = t_match;
top_match = t_match;
for i=1:length(t_imu)
    [~, index] = min(abs(t_jig-t_imu(i)));
    t_match(i) = t_jig(index);
    base_match(i) = b_angle(index);
    top_match(i) = t_angle(index);
end



end

