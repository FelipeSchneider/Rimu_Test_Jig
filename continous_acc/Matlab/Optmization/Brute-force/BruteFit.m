function [ kte_madgwick, kte_CF, kte_mahony, i, j, rms_CF, rms_madgwick, ...
    rms_mahony, top_limit, step, steps, steps_CF ] = BruteFit( acc_input, gyro_input, mag_input )
%addpath(genpath('C:\Users\felip\Documents\Arquivos dissertação\Testes dissertação\data colection'))  %just to be able to load previously jig measurements
addpath(genpath('C:\Users\felip\Documents\Arquivos dissertação\Testes dissertação\_new data collection'))
addpath(genpath('..\..\fusion'));
addpath('..\..\jig');
addpath('..\..\');
%clc;
%load random2

%load mid_range_data2.mat   %o 6 ficou com erro baixo
% load slow_range_data.mat
load fast_range_data4.mat 
clearvars -except com_data description fs   ...
    jig_const  Q real_base_angle real_top_angle t...
    t_angles t_imu t_rec acc_input gyro_input mag_input

load LIS_cal_mat
load Yaw_alignment
load Pitch_alignment

 close all; 

%% Data that must be pre entered
Rot_Order = 'ZXY';
acc_data = acc_input;
gyro_data = gyro_input;
mag_data = mag_input;
%mag_data = mag_bno_gaus;
avg_time_quat = 200:350;
avg_time_sens = 200:350;

fs = 100;                                   %sampling rate
%the test must start with a steady period
bw = mean(gyro_data(:,avg_time_sens),2);       %gyro bias
fn = mean(acc_data(:,avg_time_sens),2);          %gravity in the sensor frame
mn = mean(mag_data(:,avg_time_sens),2);       %magnetic field in the sensor frame


%% Jig time ajustment
t_rec(1) = [];                  %the first position is a zero, used only for prealocation
t_expect = com_data(1,:);       %extracting the expect time for each command
t_expect(end+1) = t(end);       %filling with the end of all comands here
t = t-t_rec(1);

ratio = t_rec./t_expect;
ww=2;
t_jig_comp = t;
for i=1:length(t)
    if(t(i)>t_expect)
        ww = ww+1;
    end
    t_jig_comp(i) = t(i)*ratio(ww);
end
%just to plot the marks on the graph
b_angle_expect = com_data(2,:); b_angle_expect(end+1) = real_base_angle(end);
t_angle_expect = com_data(3,:); t_angle_expect(end+1) = real_top_angle(end);

%wrap the aligned angles around -180 and 180 degrees
b_angle = wrapTo180(real_base_angle); b_angle_expect = wrapTo180(b_angle_expect);
t_angle = wrapTo180(real_top_angle);  t_angle_expect = wrapTo180(t_angle_expect);

%match the imu and jig times
[t_match, base_match, top_match] = matchJigIMU(t_jig_comp, t_imu, b_angle, t_angle);
%base_match = -base_match;
q_jig = zeros(length(t_match),4);
for i=1:length(top_match)
    q_jig(i,:) = angle2quat(base_match(i)*pi/180, 0, -top_match(i)*pi/180, Rot_Order);
end
[e_jig(:,1),e_jig(:,2),e_jig(:,3)] = quat2angle(q_jig, Rot_Order);


%% Brute force algorithm 
step = 0.01;
top_limit = 3;
j=1;
for i=0.01:step:top_limit
%DFM implementation
q_madgwick = madgwickAlgorithm(acc_data, gyro_data, mag_data, fs, bw, i,[-0.8009, 0.0005, -0.0177, 0.5985]);
q_mahony = mahonyAlgorithm(acc_data, gyro_data, mag_data, fs, bw, i,[-0.8059, -0.0012, -0.0189,  0.5918]);
q_CF = CF_iNemo(acc_data, gyro_data, mag_data, fs, bw, 0.2, 0.2, i/top_limit-0.001,...
    [-0.8010; 0.0010; -0.0172; 0.5984]); q_CF = q_CF';

%initial Jig orientation alignment 
q_zero_CF = avg_quaternion_markley(q_CF(avg_time_quat,:));
q_zero_madgwick = avg_quaternion_markley(q_madgwick(avg_time_quat,:));
q_zero_mahony = avg_quaternion_markley(q_mahony(avg_time_quat,:));

q_CF = quatmultiply(quatconj(q_zero_CF'),q_CF);
q_madgwick = quatmultiply(quatconj(q_zero_madgwick'),q_madgwick);
q_mahony = quatmultiply(quatconj(q_zero_mahony'),q_mahony);

%quaternion to euler conversion
[e_madgwick(:,1), e_madgwick(:,2), e_madgwick(:,3)] = quat2angle(q_madgwick, Rot_Order);
[e_mahony(:,1), e_mahony(:,2), e_mahony(:,3)] = quat2angle(q_mahony, Rot_Order);
[e_CF(:,1), e_CF(:,2), e_CF(:,3)] = quat2angle(q_CF, Rot_Order);

%correction for the yaw 
[ e_madgwick ] = angleAlignment( e_madgwick, e_jig, pol_yaw, pol_roll, pol_pitch , 'yaw');
[ e_mahony ] = angleAlignment( e_mahony, e_jig, pol_yaw, pol_roll, pol_pitch , 'yaw');
[ e_CF ] = angleAlignment( e_CF, e_jig, pol_yaw, pol_roll, pol_pitch , 'yaw');

%correction for the pitch 
[ e_madgwick ] = angleAlignment( e_madgwick, e_jig, pitch_pol_yaw, pitch_pol_roll, pitch_pol_pitch , 'pitch');
[ e_mahony ] = angleAlignment( e_mahony, e_jig, pitch_pol_yaw, pitch_pol_roll, pitch_pol_pitch , 'pitch');
[ e_CF ] = angleAlignment( e_CF, e_jig, pitch_pol_yaw, pitch_pol_roll, pitch_pol_pitch , 'pitch');

[ ~, e_madgwick, e_mahony, e_CF, ~ ] ...
    = jigTimeAlign( zeros(size(e_madgwick)), e_madgwick, e_mahony, e_CF, zeros(size(e_madgwick)), e_jig );

[kte_madgwick(j),rms_madgwick(j)] = calc_Fit(e_madgwick,e_jig);
[kte_mahony(j),rms_mahony(j)] = calc_Fit(e_mahony,e_jig);
[kte_CF(j),rms_CF(j)] = calc_Fit(e_CF,e_jig);

fprintf('Iteration number %d \r',j);
fprintf('Fits KTE, CF: %f   Madgwick: %f   Mahony: %f\r',kte_CF(j),kte_madgwick(j),kte_mahony(j));
fprintf('Fits RMS, CF: %f   Madgwick: %f   Mahony: %f\r',rms_CF(j),rms_madgwick(j),rms_mahony(j));

steps(j) = i; 
steps_CF(j) = i/top_limit-0.01;
j = j + 1;

% figure(16); 
% subplot(311);plot(base_match,'b');hold all;
% plot(e_CF(:,1)*180/pi,'r')
% plot(e_madgwick(:,1)*180/pi,'g')
% plot(e_mahony(:,1)*180/pi,'c');
% ylabel('yaw [º]'); grid on; legend('Jig','CF','Madgwick','Mahony')
% subplot(312);plot(zeros(size(base_match)));hold all;
% plot(e_CF(:,2)*180/pi,'r')
% plot(e_madgwick(:,2)*180/pi,'g')
% plot(e_mahony(:,2)*180/pi,'c');ylabel('roll [º]'); grid on;
% subplot(313);plot(-top_match);hold all;
% plot(e_CF(:,3)*180/pi,'r')
% plot(e_madgwick(:,3)*180/pi,'g')
% plot(e_mahony(:,3)*180/pi,'c');ylabel('pitch [º]'); grid on; drawnow;
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

%save('fit_lsm_lis_mag','kte_madgwick', 'kte_CF', 'kte_mahony', 'i', 'j', 'rms_CF', 'rms_madgwick', 'rms_mahony', 'top_limit', 'step', 'steps', 'steps_CF')



end

