addpath('C:\Users\felip\Dropbox\Mestrado\Dissertação\Coletas Jiga\Teste_giro_filt')  %just to be able to load previously jig measurements
addpath(genpath('fusion'));
addpath('jig');
addpath('aquire_rimu');
load fusion5
clearvars -except acc_bno_g acc_imu_g com_data description fs giro_bno_dps giro_imu_dps ...
    jig_const mag_bno_gaus mag_imu_gaus Q real_base_angle real_top_angle t...
    t_angles t_imu t_rec
%% Data that must be pre entered
%Kalman constants
P=zeros(6,6);                               %Initial Covariance matrix
P(1:3,1:3)=diag([1e-2, 1e-2, 1e-2]);        %Initial Covariance matrix
P(4:6,4:6)=diag([1e-4, 1e-4, 1e-4]);        %Initial Covariance matrix
bw = [2.5052;   -5.7301;   -4.8365]*pi/180; %gyro bias
fn = mean(acc_imu_g(:,100:400),2);          %gravity in the sensor frame
mn = mean(mag_bno_gaus(:,100:400),2);       %magnetic field in the sensor frame
R = diag([1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1]);

%Offsets and variations
gyro_offset=[2.809936e+00,-5.056333e+00,-3.587206e+00]';
gyro_var=[(1.934282e-01/180*pi)^2 (1.887641e-01/180*pi)^2 (4.747390e-01/180*pi)^2]';
%magnetometer calibration
[mag_imu_gaus_cal, Ca_imu, Cb_imu] = magCalibration(mag_imu_gaus');
mag_imu_gaus_cal = mag_imu_gaus_cal';
%% Comparing the results for the BNO
%compensating the jig time
t_rec(1) = [];                  %the first position is a zero, used only for prealocation
t_expect = com_data(1,:);       %extracting the expect time for each command
t_expect(end+1) = t(end);       %filling with the end of all comands here
t = t-t_rec(1);

%comp_factor = 123.5/123.0383;    %took from graph FUSION4
comp_factor = 56.7/56.8678;     %took from graph FUSION5
%comp_factor = 254.13/253.07818;                   %FUSION 6
t_jig_comp = t*comp_factor;

%just to plot the marks on the graph
b_angle_expect = com_data(2,:); b_angle_expect(end+1) = real_base_angle(end);
t_angle_expect = com_data(3,:); t_angle_expect(end+1) = real_top_angle(end);

%align the board with the jig
q_bno = quatnormalize(Q');

%wrap the aligned angles around -180 and 180 degrees
b_angle = wrapTo180(real_base_angle); b_angle_expect = wrapTo180(b_angle_expect);
t_angle = wrapTo180(real_top_angle);  t_angle_expect = wrapTo180(t_angle_expect);
%e_bno(end,:) = []; b_angle(end) = [];  t_angle(end) = [];

%match the imu and jig times
[t_match, base_match, top_match] = matchJigIMU(t_jig_comp, t_imu, b_angle, t_angle);
q_jig = zeros(length(t_match),4);
for i=1:length(top_match)
    q_jig(i,:) = angle2quat(base_match(i)*pi/180, 0, -top_match(i)*pi/180,'ZXY');
end
 [e_jig(:,1),e_jig(:,2),e_jig(:,3)] = quat2angle(q_jig,'ZXY');
 figure(1); plot(e_jig);

%% comparing the results for the MINIMU -- FOR NOW WITH BNO'S MAGNETOMER 
%applying the fusion algorithms
q_CF = CF_iNemo(acc_imu_g, giro_imu_dps, mag_bno_gaus, 100, gyro_offset,...
                            0.1, 0.1, .9, [1;0;0;0]); q_CF = q_CF';
q_madgwick = madgwickAlgorithm(acc_imu_g, giro_imu_dps, mag_bno_gaus, 100, ...
                            gyro_offset, 1.25);
q_mahony = mahonyAlgorithm(acc_imu_g, giro_imu_dps, mag_bno_gaus, 100, ...
                            gyro_offset, 3);
[q_gyroLib, P, bw_] = Gyro_lib_quat(P, bw, giro_imu_dps*(pi/180*(1/fs)), 1e-4, 1e-7,...
    acc_imu_g, mag_bno_gaus, fn, mn, 1/fs, R);                        
                        
%ajusting their references to the same as the jigs reference
%calculating the initial quaternion
q_zero_bno = avg_quaternion_markley(q_bno(150:450,:));
q_zero_CF = avg_quaternion_markley(q_CF(380:480,:));
q_zero_madgwick = avg_quaternion_markley(q_madgwick(380:480,:));
q_zero_mahony = avg_quaternion_markley(q_mahony(380:480,:));
q_zero_gyroLib = avg_quaternion_markley(q_gyroLib(380:480,:));

%subtracting the zero position from the quaternions
q_bno = quatmultiply(quatconj(q_zero_bno'), q_bno); q_bno = quatnormalize(q_bno);
q_CF = quatmultiply(quatconj(q_zero_CF'),q_CF);
q_madgwick = quatmultiply(quatconj(q_zero_madgwick'),q_madgwick);
q_mahony = quatmultiply(quatconj(q_zero_mahony'),q_mahony);
q_gyroLib = quatmultiply(quatconj(q_zero_gyroLib'),q_gyroLib);

%converting from quaternions to euler
[e_bno(:,1),e_bno(:,2),e_bno(:,3)] = quat2angle(q_bno,'ZXY');
[e_madgwick_imu(:,1), e_madgwick_imu(:,2), e_madgwick_imu(:,3)] = quat2angle(q_madgwick,'ZXY');
[e_mahony_imu(:,1), e_mahony_imu(:,2), e_mahony_imu(:,3)] = quat2angle(q_mahony,'ZXY');
[e_CF_imu(:,1), e_CF_imu(:,2), e_CF_imu(:,3)] = quat2angle(q_CF,'ZXY');
[e_gyroLib(:,1), e_gyroLib(:,2), e_gyroLib(:,3)] = quat2angle(q_gyroLib,'ZXY');

%% Erros
%calculate the quaternion difference btw the BNO and jig
q_jig = quatnormalize(q_jig);
% q_diff_bno = quatmultiply(quatconj(q_bno),q_jig); %q_bno
% [e_diff_bno(:,1),e_diff_bno(:,2),e_diff_bno(:,3)] = quat2angle(q_diff_bno,'ZXY');
% figure(2); plot(e_diff_bno);
% for i=1:length(e_diff_bno)
%     e_diff_bno(i,1) = angdiff(base_match(i)*pi/180,e_bno(i,1));
%     e_diff_bno(i,2) = angdiff(0,e_bno(i,2));
%     e_diff_bno(i,3) = angdiff(-top_match(i)*pi/180,e_bno(i,3));
% end

e_diff_bno(:,1) = angdiff(base_match'*pi/180,e_bno(:,1));
e_diff_bno(:,2) = angdiff(0,e_bno(:,2));
e_diff_bno(:,3) = angdiff(-top_match'*pi/180,e_bno(:,3));
e_diff_bno(isnan(e_diff_bno)) = 0;
%% Plots
figure(3); plot(t_match,e_diff_bno*180/pi); legend('Yaw','Roll','Pitch'); ylabel('Angle [°]');
xlabel('time [s]'); axis([t_match(1) inf -inf inf]); title('Angle Error');

figure(4); 
subplot(311);plot(t_imu,e_bno(:,1)*180/pi); hold all; plot(t_match,base_match); plot(t_match,e_diff_bno(:,1)*180/pi); axis([0 inf -200 200]); grid on;
title('Time matched'); ylabel('yaw'); legend('BNO','Jig','error');
subplot(312);plot(t_imu,e_bno(:,2)*180/pi);hold all; plot(t_imu,zeros(size(e_bno(:,2)))); plot(t_match,e_diff_bno(:,2)*180/pi); 
axis([0 inf -inf inf]); grid on; ylabel('roll');
subplot(313);plot(t_imu,e_bno(:,3)*180/pi);hold all; plot(t_match,-top_match); plot(t_match,e_diff_bno(:,3)*180/pi);axis([0 inf -200 200]); grid on;
ylabel('pitch');

figure(5);
subplot(311);
plot(t_match,base_match);hold all; plot(t_imu,[e_bno(:,1) e_madgwick_imu(:,1) e_mahony_imu(:,1) e_CF_imu(:,1)]*180/pi);
axis([0 inf -200 200]); grid on; title('Fusion comparison'); ylabel('yaw');
legend('Jig','BNO','Madgwick','Mahony','CF');
subplot(312);
plot(t_imu,zeros(size(e_bno(:,2))));hold all; plot(t_imu,[e_bno(:,2) e_madgwick_imu(:,2) e_mahony_imu(:,2) e_CF_imu(:,2)]*180/pi);
axis([0 inf -inf inf]); grid on; ylabel('roll');
subplot(313);
plot(t_match,-top_match);hold all; plot(t_imu,[e_bno(:,3) e_madgwick_imu(:,3) e_mahony_imu(:,3) e_CF_imu(:,3)]*180/pi);
axis([0 inf -200 200]); grid on; ylabel('pitch');
