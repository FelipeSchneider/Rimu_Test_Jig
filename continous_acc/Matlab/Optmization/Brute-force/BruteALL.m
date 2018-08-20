%addpath(genpath('C:\Users\felip\Documents\Arquivos dissertação\Testes dissertação\data colection'))  %just to be able to load previously jig measurements
addpath(genpath('C:\Users\felip\Documents\Arquivos dissertação\Testes dissertação\_new data collection'))
addpath(genpath('..\..\fusion'));
addpath('..\..\jig');
addpath('..\..\');
clc;
%load random2

% load mid_range_data2.mat   %o 6 ficou com erro baixo
% load slow_range_data.mat 
load fast_range_data4.mat 

clearvars -except acc_bno_g acc_imu_g com_data description fs giro_bno_dps giro_imu_dps ...
    jig_const mag_bno_gaus mag_imu_gaus Q real_base_angle real_top_angle t...
    t_angles t_imu t_rec

load LIS_cal_mat
load Yaw_alignment
load Pitch_alignment

%close all; 
%% calibrate LSM gyro data
giro_imu_dps = giro_imu_dps - mean(giro_imu_dps(:,100:400),2);
giro_imu_dps = giro_imu_dps*1.1692104;

%% calibrate LIS data
mag_imu_gaus_cal = mag_imu_gaus' * Ca_imu' + repmat(Cb_imu', length(mag_imu_gaus), 1);
mag_imu_gaus_cal = mag_imu_gaus_cal';


% disp('Fit 1');
% acc_input = acc_bno_g;
% gyro_input = giro_bno_dps;
% mag_input = mag_bno_gaus;
% [ kte_madgwick, kte_CF, kte_mahony, i, j, rms_CF, rms_madgwick, ...
%     rms_mahony, top_limit, step, steps, steps_CF ] = BruteFit( acc_input, gyro_input, mag_input );
% save('fit_1','kte_madgwick', 'kte_CF', 'kte_mahony', 'i', 'j', ...
%     'rms_CF', 'rms_madgwick', 'rms_mahony', 'top_limit', 'step', 'steps', 'steps_CF')
% 
% disp('Fit 2');
% acc_input = acc_imu_g;
% gyro_input = giro_imu_dps;
% mag_input = mag_imu_gaus_cal;
% [ kte_madgwick, kte_CF, kte_mahony, i, j, rms_CF, rms_madgwick, ...
%     rms_mahony, top_limit, step, steps, steps_CF ] = BruteFit( acc_input, gyro_input, mag_input );
% save('fit_2','kte_madgwick', 'kte_CF', 'kte_mahony', 'i', 'j', ...
%     'rms_CF', 'rms_madgwick', 'rms_mahony', 'top_limit', 'step', 'steps', 'steps_CF')
% 
% disp('Fit 3');
% acc_input = acc_bno_g;
% gyro_input = giro_bno_dps;
% mag_input = mag_imu_gaus_cal;
% [ kte_madgwick, kte_CF, kte_mahony, i, j, rms_CF, rms_madgwick, ...
%     rms_mahony, top_limit, step, steps, steps_CF ] = BruteFit( acc_input, gyro_input, mag_input );
% save('fit_3','kte_madgwick', 'kte_CF', 'kte_mahony', 'i', 'j', ...
%     'rms_CF', 'rms_madgwick', 'rms_mahony', 'top_limit', 'step', 'steps', 'steps_CF')
% 
% disp('Fit 4');
% acc_input = acc_imu_g;
% gyro_input = giro_bno_dps;
% mag_input = mag_imu_gaus_cal;
% [ kte_madgwick, kte_CF, kte_mahony, i, j, rms_CF, rms_madgwick, ...
%     rms_mahony, top_limit, step, steps, steps_CF ] = BruteFit( acc_input, gyro_input, mag_input );
% save('fit_4','kte_madgwick', 'kte_CF', 'kte_mahony', 'i', 'j', ...
%     'rms_CF', 'rms_madgwick', 'rms_mahony', 'top_limit', 'step', 'steps', 'steps_CF')

disp('Fit 5');
acc_input = acc_bno_g;
gyro_input = giro_imu_dps;
mag_input = mag_imu_gaus_cal;
[ kte_madgwick, kte_CF, kte_mahony, i, j, rms_CF, rms_madgwick, ...
    rms_mahony, top_limit, step, steps, steps_CF ] = BruteFit( acc_input, gyro_input, mag_input );
save('fit_5','kte_madgwick', 'kte_CF', 'kte_mahony', 'i', 'j', ...
    'rms_CF', 'rms_madgwick', 'rms_mahony', 'top_limit', 'step', 'steps', 'steps_CF')

disp('Fit 6');
acc_input = (acc_bno_g+acc_imu_g)/2;
gyro_input = (giro_bno_dps+giro_imu_dps)/2;
mag_input = mag_bno_gaus;
[ kte_madgwick, kte_CF, kte_mahony, i, j, rms_CF, rms_madgwick, ...
    rms_mahony, top_limit, step, steps, steps_CF ] = BruteFit( acc_input, gyro_input, mag_input );
save('fit_6','kte_madgwick', 'kte_CF', 'kte_mahony', 'i', 'j', ...
    'rms_CF', 'rms_madgwick', 'rms_mahony', 'top_limit', 'step', 'steps', 'steps_CF')

disp('Fit 7');
acc_input = (acc_bno_g+acc_imu_g)/2;
gyro_input = (giro_bno_dps+giro_imu_dps)/2;
mag_input = mag_imu_gaus_cal;
[ kte_madgwick, kte_CF, kte_mahony, i, j, rms_CF, rms_madgwick, ...
    rms_mahony, top_limit, step, steps, steps_CF ] = BruteFit( acc_input, gyro_input, mag_input );
save('fit_7','kte_madgwick', 'kte_CF', 'kte_mahony', 'i', 'j', ...
    'rms_CF', 'rms_madgwick', 'rms_mahony', 'top_limit', 'step', 'steps', 'steps_CF')

disp('Fit 8');
acc_input = (acc_bno_g+acc_imu_g)/2;
gyro_input = (giro_bno_dps+giro_imu_dps)/2;
mag_input = (mag_imu_gaus_cal+mag_bno_gaus)/2;
[ kte_madgwick, kte_CF, kte_mahony, i, j, rms_CF, rms_madgwick, ...
    rms_mahony, top_limit, step, steps, steps_CF ] = BruteFit( acc_input, gyro_input, mag_input );
save('fit_8','kte_madgwick', 'kte_CF', 'kte_mahony', 'i', 'j', ...
    'rms_CF', 'rms_madgwick', 'rms_mahony', 'top_limit', 'step', 'steps', 'steps_CF')






