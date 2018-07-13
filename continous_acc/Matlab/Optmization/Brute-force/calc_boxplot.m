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

%% data from scenario 1
% Data that must be pre entered
% acc_data = acc_bno_g;
% giro_data = giro_bno_dps;
% mag_data = mag_bno_gaus;
% cross_CF = 0.983;
% Kp_mahony = 0.455;
% Beta_madgwick = 0.03;

%% data from scenario 2
% acc_data = acc_imu_g;
% giro_data = giro_imu_dps;
% mag_data = mag_imu_gaus_cal;
% cross_CF = 0.983;
% Kp_mahony = 1.455;
% Beta_madgwick = 0.055;

%% data from scenario 3
% acc_data = acc_bno_g;
% giro_data = giro_bno_dps;
% mag_data = mag_imu_gaus_cal;
% cross_CF = 0.983;
% Kp_mahony = 1.905;
% Beta_madgwick = 0.155;

%% data from scenario 4
% acc_data = acc_imu_g;
% giro_data = giro_bno_dps;
% mag_data = mag_imu_gaus_cal;
% cross_CF = 0.983;
% Kp_mahony = 1.255;
% Beta_madgwick = 0.055;

%% data from scenario 5
% acc_data = acc_bno_g;
% giro_data = giro_imu_dps;
% mag_data = mag_imu_gaus_cal;
% cross_CF = 0.983;
% Kp_mahony = 2.105;
% Beta_madgwick = 0.255;

%% data from scenario 6
% acc_data = (acc_imu_g + acc_bno_g)/2;
% giro_data = (giro_imu_dps + giro_bno_dps)/2;
% mag_data = mag_bno_gaus;
% cross_CF = 0.983;
% Kp_mahony = 0.505;
% Beta_madgwick = 0.055;

%% data from scenario 7
acc_data = (acc_imu_g + acc_bno_g)/2;
giro_data = (giro_imu_dps + giro_bno_dps)/2;
mag_data = mag_imu_gaus_cal;
cross_CF = 0.983;
Kp_mahony = 1.630;
Beta_madgwick = 0.155;

%% initial sensor readings
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


%% fusion algorithms




q_CF = CF_iNemo(acc_data, giro_data, mag_data, fs, bw, 0.2, 0.2, cross_CF,...
    [-0.8893; -0.0108; -0.0579; 0.4535]); q_CF = q_CF';
q_madgwick = madgwickAlgorithm(acc_data, giro_data, mag_data, fs, bw, Beta_madgwick,[0.7915, 0.0191, 0.0534, -0.6085]);
q_mahony = mahonyAlgorithm(acc_data, giro_data, mag_data, fs, bw, Kp_mahony,[-0.7922, -0.0202, -0.0547, 0.6074]);

q_zero_CF = avg_quaternion_markley(q_CF(380:480,:));
q_zero_madgwick = avg_quaternion_markley(q_madgwick(380:480,:));
q_zero_mahony = avg_quaternion_markley(q_mahony(380:480,:));

q_CF = quatmultiply(quatconj(q_zero_CF'),q_CF);
q_madgwick = quatmultiply(quatconj(q_zero_madgwick'),q_madgwick);
q_mahony = quatmultiply(quatconj(q_zero_mahony'),q_mahony);

[e_CF_7] = calc_diff(q_CF,q_jig);
[e_madgwick_7] = calc_diff(q_madgwick,q_jig);
[e_mahony_7] = calc_diff(q_mahony,q_jig);

save('error_7','e_CF_7', 'e_madgwick_7', 'e_mahony_7');
% [kte_CF(j),rms_CF(j)] = calc_Fit(q_CF,q_jig);
% [kte_madgwick(j),rms_madgwick(j)] = calc_Fit(q_madgwick,q_jig);
% [kte_mahony(j),rms_mahony(j)] = calc_Fit(q_mahony,q_jig);


