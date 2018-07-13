addpath(genpath('C:\Users\felip\Documents\Arquivos dissertação\Testes dissertação\data colection'))  %just to be able to load previously jig measurements
addpath(genpath('..\..\fusion'));
addpath('..\..\jig');
addpath('..\..\');
clc;
load random2
clearvars -except acc_bno_g acc_imu_g com_data description fs giro_bno_dps giro_imu_dps ...
    jig_const mag_bno_gaus mag_imu_gaus Q real_base_angle real_top_angle t...
    t_angles t_imu t_rec
load LIS_calibration_matrices
 close all; 
%% calibrate LSM gyro data
giro_imu_dps = giro_imu_dps - mean(giro_imu_dps(:,100:400),2);
giro_imu_dps = giro_imu_dps*1.1692104;

%% calibrate LIS data
mag_imu_gaus_cal = mag_imu_gaus' * Ca_imu' + repmat(Cb_imu', length(mag_imu_gaus), 1);
mag_imu_gaus_cal = mag_imu_gaus_cal';
%% Data that must be pre entered
acc_data = acc_imu_g;
giro_data = giro_imu_dps;
mag_data = mag_imu_gaus_cal;

fs = 100;                                   %sampling rate
bw = mean(giro_data(:,100:400),2);          %gyro bias
fn = mean(acc_data(:,100:400),2);           %gravity in the sensor frame
mn = mean(mag_data(:,100:400),2);           %magnetic field in the sensor frame


%% Compensating the jig time and 
t_rec(1) = [];                  %the first position is a zero, used only for prealocation
t_expect = com_data(1,:);       %extracting the expect time for each command
t_expect(end+1) = t(end);       %filling with the end of all comands here
t = t-t_rec(1);

comp_factor = t_rec(end)/t_expect(end);
%comp_factor = 123.5/123.0383;    %took from graph FUSION4
%comp_factor = 56.7/56.8678;     %took from graph FUSION5
%comp_factor = 254.13/253.07818;                   %FUSION 6
t_jig_comp = t*comp_factor;

%just to plot the marks on the graph
b_angle_expect = com_data(2,:); b_angle_expect(end+1) = real_base_angle(end);
t_angle_expect = com_data(3,:); t_angle_expect(end+1) = real_top_angle(end);

%wrap the aligned angles around -180 and 180 degrees
b_angle = wrapTo180(real_base_angle); b_angle_expect = wrapTo180(b_angle_expect);
t_angle = wrapTo180(real_top_angle);  t_angle_expect = wrapTo180(t_angle_expect);

%match the imu and jig times
[t_match, base_match, top_match] = matchJigIMU(t_jig_comp, t_imu, b_angle, t_angle);
q_jig = zeros(length(t_match),4);
for i=1:length(top_match)
    q_jig(i,:) = angle2quat(base_match(i)*pi/180, 0, -top_match(i)*pi/180,'ZXY');
end


%% Brute force algorithm 
step = 0.025;
top_limit = 3;
j=1;
for i=0.03:step:top_limit
q_CF = CF_iNemo(acc_data, giro_data, mag_data, fs, bw, 0.2, 0.2, i/top_limit-0.01,...
    [-0.8893; -0.0108; -0.0579; 0.4535]); q_CF = q_CF';
q_madgwick = madgwickAlgorithm(acc_data, giro_data, mag_data, fs, bw, i,[0.7915, 0.0191, 0.0534, -0.6085]);
q_mahony = mahonyAlgorithm(acc_data, giro_data, mag_data, fs, bw, i,[-0.7922, -0.0202, -0.0547, 0.6074]);

q_zero_CF = avg_quaternion_markley(q_CF(380:480,:));
q_zero_madgwick = avg_quaternion_markley(q_madgwick(380:480,:));
q_zero_mahony = avg_quaternion_markley(q_mahony(380:480,:));

q_CF = quatmultiply(quatconj(q_zero_CF'),q_CF);
q_madgwick = quatmultiply(quatconj(q_zero_madgwick'),q_madgwick);
q_mahony = quatmultiply(quatconj(q_zero_mahony'),q_mahony);

[kte_CF(j),rms_CF(j)] = calc_Fit(q_CF,q_jig);
[kte_madgwick(j),rms_madgwick(j)] = calc_Fit(q_madgwick,q_jig);
[kte_mahony(j),rms_mahony(j)] = calc_Fit(q_mahony,q_jig);

fprintf('Iteration number %d \r',j);
fprintf('Fits KTE, CF: %f   Madgwick: %f   Mahony: %f\r',kte_CF(j),kte_madgwick(j),kte_mahony(j));
fprintf('Fits RMS, CF: %f   Madgwick: %f   Mahony: %f\r',rms_CF(j),rms_madgwick(j),rms_mahony(j));

steps(j) = i; 
steps_CF(j) = i/top_limit-0.01;
j = j + 1;


[e_cf(:,1),e_cf(:,2),e_cf(:,3)] = quat2angle(q_CF,'ZXY');
[e_madg(:,1),e_madg(:,2),e_madg(:,3)] = quat2angle(q_madgwick,'ZXY');
[e_mahony(:,1),e_mahony(:,2),e_mahony(:,3)] = quat2angle(q_mahony,'ZXY');
% figure(15); 
% subplot(311);plot(base_match);hold on;plot([e_madg(:,1),e_mahony(:,1)]*180/pi);%plot(e_diff(:,1));
% ylabel('yaw [º]'); grid on;
% subplot(312);plot(zeros(size(base_match)));hold on;plot([e_madg(:,2),e_mahony(:,2)]*180/pi);%plot(e_diff(:,2));
% ylabel('roll [º]'); grid on;
% subplot(313);plot(-top_match);hold on;plot([e_madg(:,3),e_mahony(:,3)]*180/pi);%plot(e_diff(:,3));
% ylabel('pitch [º]'); grid on;
% 
% figure(16); 
% subplot(311);plot(base_match);hold on;plot([e_cf(:,1),e_madg(:,1),e_mahony(:,1)]*180/pi);%plot(e_diff(:,1));
% ylabel('yaw [º]'); grid on; legend('Jig','CF','Madgwick','Mahony')
% subplot(312);plot(zeros(size(base_match)));hold on;plot([e_cf(:,2),e_madg(:,2),e_mahony(:,2)]*180/pi);%plot(e_diff(:,2));
% ylabel('roll [º]'); grid on;
% subplot(313);plot(-top_match);hold on;plot([e_cf(:,3),e_madg(:,3),e_mahony(:,3)]*180/pi);%plot(e_diff(:,3));
% ylabel('pitch [º]'); grid on;
end
[best_CF, i_best_CF] = min(kte_CF);
[best_madgwick, i_best_madgwick] = min(kte_madgwick);
[best_mahony, i_best_mahony] = min(kte_mahony);

fprintf('Best CF %f \r',steps_CF(i_best_CF)); 
fprintf('Best Madgwick %f \r',steps(i_best_madgwick)); 
fprintf('Best Mahony %f \r',steps(i_best_mahony)); 


figure;
plot(steps_CF,kte_CF); hold on;
plot(steps, [kte_madgwick; kte_mahony]'); 
legend('CF','Madgwick','Mahony');
title('Fit KTE');

figure;
plot(steps_CF,rms_CF); hold on;
plot(steps, [rms_madgwick; rms_mahony]'); 
legend('CF','Madgwick','Mahony');
title('Fit RMS');

% figure;
% plot([rms_CF; rms_madgwick; rms_mahony]'); legend('CF','Madgwick','Mahony');
% title('Fit RMS');

save('fit_lsm_lis_mag','kte_madgwick', 'kte_CF', 'kte_mahony', 'i', 'j', 'rms_CF', 'rms_madgwick', 'rms_mahony', 'top_limit', 'step', 'steps', 'steps_CF')

