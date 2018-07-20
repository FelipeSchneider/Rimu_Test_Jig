addpath('..\');
clear all; close all;
%% initialization and file load
%This script is intended to compare the performance of many different
%fusion algorithms. 
%... In construction

%data from tests during the magnetometer calibration
% addpath('mag_calibration');
% addpath('mag_calibration\data');      % include quaternion library
% load('cal_data_1.mat');     %BNO was previously calibrated
% load('cal_data_2.mat');   %BNO was previously calibrated
% load('cal_data_5.mat');   %BNO was previously calibrated

% data from tests after the magnetometer calibration that already have the
% calibration matrix calculated and loaded by cal_matrix_x
% load('cal_matrix_1.mat'); load('teste_1_1.mat');
%load('cal_matrix_2.mat'); %load('teste_2_1.mat');
 
%  load('PRY_lock_2.mat');
% load('PRY_lock.mat');
% load('PRY_normal_2.mat');
% load('PRY_lock_3.mat');
%load('PRY_lock_4.mat');

load slow_range_data

%% Magnetometer calibration
if( exist('Ca_imu','var') == 1)
    mag_imu_gaus_cal = mag_imu_gaus' * Ca_imu' + repmat(Cb_imu', length(mag_imu_gaus), 1);
    disp('There was a calibration matrix for the Minimu');
else
    [mag_imu_gaus_cal, Ca_imu, Cb_imu] = magCalibration(mag_imu_gaus');
    disp('There was NOT a calibration matrix for the Minimu');
    disp('A new calibration matrix is being calculated');
end
mag_imu_gaus_cal = mag_imu_gaus_cal';

%% Acelerometer and gyro offsets and variation:
gyro_offset=[2.809936e+00,-5.056333e+00,-3.587206e+00]';
gyro_var=[(1.934282e-01/180*pi)^2 (1.887641e-01/180*pi)^2 (4.747390e-01/180*pi)^2]';

%% Kalman with Descendent Gradient
addpath('Kalman_iNemo\Gradient-Descendent');      % include quaternion library
% q_GD_imu = Kalman_GD(acc_imu_g, giro_imu_dps, mag_imu_gaus_cal, 100,...
%                      gyro_offset, gyro_var, .1, .1, 0.2, 0.2, [1;0;0;0]);

q_GD_imu = Kalman_GD(acc_imu_g, giro_imu_dps, mag_bno_gaus, 100,...
                     gyro_offset, gyro_var, 0.025, 0.025, 0.25, 0.25, [1;0;0;0]);
                                   
%% Kalman with Gauss-Newton
addpath('Kalman_iNemo\Gauss-Newton');      % include quaternion library
q_GN_imu = Kalman_GN(acc_imu_g, giro_imu_dps, mag_bno_gaus, 100,...
                     gyro_offset, gyro_var, 0.025, 0.025, 0.25, 0.25, [1;0;0;0]);

%% Complementary filters provided by the iNemo group
addpath('CF_iNemo');
q_CF = CF_iNemo(acc_imu_g, giro_imu_dps, mag_bno_gaus, 100, gyro_offset,...
                            0.1, 0.1, .9, [1;0;0;0]);
%% Madgwick Algorithm
addpath('Madgwick');
addpath('Madgwick\quaternion_library');     
q_madgwick = madgwickAlgorithm(acc_imu_g, giro_imu_dps, mag_bno_gaus, 100, ...
                            gyro_offset, 1.25);

%% Mahony Algorithm
addpath('Mahony');
addpath('Madgwick\quaternion_library'); 
q_mahony = mahonyAlgorithm(acc_imu_g, giro_imu_dps, mag_bno_gaus, 100, ...
                            gyro_offset, 1.75);

%% Conversion to euler angles
[r1_bno, r2_bno, r3_bno] = quat2angle(Q','ZXY');      
[r1_GD_imu, r2_GD_imu, r3_GD_imu] = quat2angle(q_GD_imu','ZXY');
[r1_GN_imu, r2_GN_imu, r3_GN_imu] = quat2angle(q_GN_imu','ZXY');
[r1_madgwick_imu, r2_madgwick_imu, r3_madgwick_imu] = quat2angle(q_madgwick,'ZXY');
[r1_mahony_imu, r2_mahony_imu, r3_mahony_imu] = quat2angle(q_mahony,'ZXY');
[r1_CF_imu, r2_CF_imu, r3_CF_imu] = quat2angle(q_CF','ZXY');


% [r1_bno, r2_bno, r3_bno] = quat2angle(Q','XZY');      
% [r1_GD_imu, r2_GD_imu, r3_GD_imu] = quat2angle(q_GD_imu','XZY');
% [r1_GN_imu, r2_GN_imu, r3_GN_imu] = quat2angle(q_GN_imu','XZY');
% [r1_madgwick_imu, r2_madgwick_imu, r3_madgwick_imu] = quat2angle(q_madgwick,'XZY');
% [r1_mahony_imu, r2_mahony_imu, r3_mahony_imu] = quat2angle(q_mahony,'XZY');
% [r1_CF_imu, r2_CF_imu, r3_CF_imu] = quat2angle(q_CF','XZY');
%% Plots
figure(1);hold all;
subplot(311); 
plot(t_imu,[r1_bno,r1_GD_imu, r1_GN_imu, r1_madgwick_imu,r1_mahony_imu, r1_CF_imu]*180/pi'); 
title('Angulos de Euler'); hold all; ylabel('Yaw'); 
legend('BNO','IMU GD','IMU GN','IMU Madgwick','IMU Mahony','IMU CF');
grid on;

subplot(312); 
plot(t_imu,[r2_bno,r2_GD_imu, r2_GN_imu,r2_madgwick_imu,r2_mahony_imu, r2_CF_imu]*180/pi'); 
hold all; legend('BNO','IMU GD','IMU GN','IMU Madgwick','IMU Mahony','IMU CF');ylabel('Roll');
grid on;

subplot(313);  
plot(t_imu,[r3_bno,r3_GD_imu, r3_GN_imu,r3_madgwick_imu,r3_mahony_imu, r3_CF_imu]*180/pi'); 
hold all; legend('BNO','IMU GD','IMU GN','IMU Madgwick','IMU Mahony','IMU CF');ylabel('Pitch');
grid on;



figure(2);hold all;
subplot(311); 
plot(t_imu,[r1_bno,r1_madgwick_imu,r1_mahony_imu, r1_CF_imu]*180/pi'); 
title('Angulos de Euler'); hold all; ylabel('Yaw'); 
legend('BNO','IMU Madgwick','IMU Mahony','IMU CF');
grid on;

subplot(312); 
plot(t_imu,[r2_bno,r2_madgwick_imu,r2_mahony_imu, r2_CF_imu]*180/pi'); 
hold all; legend('BNO','IMU Madgwick','IMU Mahony','IMU CF');ylabel('Roll');
grid on;

subplot(313);  
plot(t_imu,[r3_bno,r3_madgwick_imu,r3_mahony_imu, r3_CF_imu]*180/pi'); 
hold all; legend('BNO','IMU Madgwick','IMU Mahony','IMU CF');ylabel('Pitch');
grid on;


figure(3); hold all;

[r1_ZYX, r2_ZYX, r3_ZYX] = quat2angle(Q','ZYX');  
[r1_ZXY, r2_ZXY, r3_ZXY] = quat2angle(Q','ZXY');  
[r1_YXZ, r2_YXZ, r3_YXZ] = quat2angle(Q','YXZ');  
[r1_YZX, r2_YZX, r3_YZX] = quat2angle(Q','YZX'); 
[r1_XYZ, r2_XYZ, r3_XYZ] = quat2angle(Q','XYZ');
[r1_XZY, r2_XZY, r3_XZY] = quat2angle(Q','XZY');

subplot(311);
plot(t_imu, r1_ZYX*180/pi,'LineWidth',2); hold all;
plot(t_imu,[r1_ZXY,r1_YXZ, r1_YZX,r1_XYZ,r1_XZY]*180/pi'); 
title('Comp. ordem de rotação'); hold all; ylabel('Yaw'); 
legend('ZYX','ZXY','YXZ','YZX','XYZ','XZY');
grid on;

subplot(312); 
plot(t_imu, r2_ZYX*180/pi,'LineWidth',2); hold all;
plot(t_imu,[r2_ZXY,r2_YXZ, r2_YZX,r2_XYZ,r2_XZY]*180/pi'); 
hold all; ylabel('Pitch');
grid on;

subplot(313);  
plot(t_imu, r3_ZYX*180/pi,'LineWidth',2); hold all;
plot(t_imu,[r3_ZXY,r3_YXZ, r3_YZX,r3_XYZ,r3_XZY]*180/pi'); 
hold all; ylabel('Roll');
grid on;


figure(4); hold all;
subplot(311);
plot(t_imu, r1_ZYX*180/pi,'LineWidth',2); hold all;
plot(t_imu,[r1_ZXY]*180/pi'); 
title('Comp. ordem de rotação'); hold all; ylabel('Yaw'); 
legend('ZYX','ZXY');
grid on;

subplot(312); 
plot(t_imu, r2_ZYX*180/pi,'LineWidth',2); hold all;
plot(t_imu,[r2_ZXY]*180/pi'); 
hold all; ylabel('Pitch');
grid on;

subplot(313);  
plot(t_imu, r3_ZYX*180/pi,'LineWidth',2); hold all;
plot(t_imu,[r3_ZXY]*180/pi'); 
hold all; ylabel('Roll');
grid on;