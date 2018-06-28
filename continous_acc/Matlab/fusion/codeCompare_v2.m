addpath('C:\Users\felip\Dropbox\Mestrado\Dissertação\Coletas Jiga\Teste_giro_filt')  %just to be able to load previously jig measurements
addpath(genpath('C:\Users\felip\Documents\Arquivos dissertação\Testes dissertação'))
addpath(genpath('..\fusion'));
addpath('..\');

load open_space_mag_cal.mat
load 5cm-trafo.mat

%% calibrate LSM gyro data
giro_imu_dps = giro_imu_dps - mean(giro_imu_dps(:,100:400),2);
giro_imu_dps = giro_imu_dps*1.1692104;

%% calibrate LIS data
mag_imu_gaus_cal = mag_imu_gaus' * Ca_imu' + repmat(Cb_imu', length(mag_imu_gaus), 1);
mag_imu_gaus_cal = mag_imu_gaus_cal';
%% Data that must be pre entered
Rot_Order = 'ZXY';
acc_data = acc_bno_g;
gyro_data = giro_bno_dps;
mag_data = mag_bno_gaus;
%Kalman constants
%X(1) -> Three values of superior diag. of R -> R(1,1);R(2,2);R(3,3). measurement noise covariance matrix
%X(2) -> Three values of inferior diag. of R -> R(4,4);R(5,5);R(6,6). measurement noise covariance matrix
%X(3) -> Three values of superior diag. of P -> P(1,1);P(2,2);P(3,3). error covariance matrix
%X(4) -> Three values of inferior diag. of P -> P(4,4);P(5,5);P(6,6). error covariance matrix
%X(5) -> gyro error noise
%X(6) -> gyro bias noise
R = diag([0.2677, 0.2677, 0.2677, 2.5, 2.5, 2.5]); %measurement noise covariance matrix
P = diag([5.17223e-05, 5.17223e-05, 5.17223e-05, 0.001111, 0.001111, 0.001111]); %error covariance matrix
Gyro_errors_noise = 0.0010139;
Gyro_errors_bias = 8.68805305601078e-07;

%the test must start with a steady period
bw = mean(gyro_data(:,100:400),2);       %gyro bias
fn = mean(acc_data(:,100:400),2);          %gravity in the sensor frame
mn = mean(mag_data(:,100:400),2);       %magnetic field in the sensor frame

%align the board with the jig
q_bno = quatnormalize(Q');

%% comparing the results for the MINIMU -- FOR NOW WITH BNO'S MAGNETOMER 
%applying the fusion algorithms
q_CF = CF_iNemo(acc_data, gyro_data, mag_data, fs, bw,...
                            0.5, 0.5, .5, [sqrt(2)/2;0;0;sqrt(2)/2]); q_CF = q_CF';
q_madgwick = madgwickAlgorithm(acc_data, gyro_data, mag_data, fs, ...
                            bw, 0.56);  %Quaternion = [-0.5911 0.0158 0.0068 -0.8064];
q_mahony = mahonyAlgorithm(acc_data, gyro_data, mag_data, fs, ...
                            bw, 4.46);  %Quaternion = [-0.5916 0.0156 0.0067 -0.8061]; 
[q_gyroLib, P, bw_] = Gyro_lib_quat(P, bw*(pi/180), gyro_data*(pi/180*(1/fs)), Gyro_errors_noise, Gyro_errors_bias,...
    acc_data, mag_data, fn, mn, 1/fs, R);   
q_gyroLib(isnan(q_gyroLib(:,1)),1) = 1; %replace NaN by unit vector, usually the last measurement is NaN
q_gyroLib(isnan(q_gyroLib(:,2)),2:4) = 0;
q_gyroLib = quatconj(q_gyroLib); %the kalman gyrolib returns the quaternion conjugate
                        
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
[e_bno(:,1),e_bno(:,2),e_bno(:,3)] = quat2angle(q_bno, Rot_Order);
[e_madgwick_imu(:,1), e_madgwick_imu(:,2), e_madgwick_imu(:,3)] = quat2angle(q_madgwick, Rot_Order);
[e_mahony_imu(:,1), e_mahony_imu(:,2), e_mahony_imu(:,3)] = quat2angle(q_mahony, Rot_Order);
[e_CF_imu(:,1), e_CF_imu(:,2), e_CF_imu(:,3)] = quat2angle(q_CF, Rot_Order);
[e_gyroLib(:,1), e_gyroLib(:,2), e_gyroLib(:,3)] = quat2angle(q_gyroLib, Rot_Order);

%% Plots
figure;
subplot(311);
plot(t,[e_bno(:,1) e_madgwick_imu(:,1) e_mahony_imu(:,1) e_CF_imu(:,1) e_gyroLib(:,1)]*180/pi);
axis([0 inf -inf inf]); grid on; title('Fusion comparison'); ylabel('yaw');
legend('BNO','Madgwick','Mahony','CF','Kalman','Orientation','Horizontal');
subplot(312);
plot(t,[e_bno(:,2) e_madgwick_imu(:,2) e_mahony_imu(:,2) e_CF_imu(:,2) e_gyroLib(:,2)]*180/pi);
axis([0 inf -inf inf]); grid on; ylabel('roll');
subplot(313);
plot(t,[e_bno(:,3) e_madgwick_imu(:,3) e_mahony_imu(:,3) e_CF_imu(:,3) e_gyroLib(:,3)]*180/pi);
axis([0 inf -inf inf]); grid on; ylabel('pitch');