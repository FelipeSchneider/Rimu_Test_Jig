addpath('C:\Users\felip\Dropbox\Mestrado\Dissertação\Coletas Jiga\Teste_giro_filt')  %just to be able to load previously jig measurements
addpath(genpath('C:\Users\felip\Documents\Arquivos dissertação\Testes dissertação'))
addpath(genpath('..\fusion'));
addpath('..\');
clear all;
load Open_space_cal_mat.mat
%load 5cm-trafo.mat
load open_space_90_degrees.mat
%% calibrate LSM gyro data
giro_imu_dps = giro_imu_dps - mean(giro_imu_dps(:,100:400),2);
giro_imu_dps = giro_imu_dps*1.1692104;

%% calibrate LIS data
%[mag_imu_gaus_cal, Ca_imu, Cb_imu] = magCalibration(mag_imu_gaus');
mag_imu_gaus_cal = mag_imu_gaus' * Ca_imu' + repmat(Cb_imu', length(mag_imu_gaus), 1);
mag_imu_gaus_cal = mag_imu_gaus_cal';
%% Data that must be pre entered
Rot_Order = 'ZXY';
acc_data = acc_bno_g;
gyro_data = giro_bno_dps;
%mag_data = mag_bno_gaus;
mag_data = mag_imu_gaus_cal;
avg_time = 2000:2100;
%Kalman constants
%X(1) -> Three values of superior diag. of R -> R(1,1);R(2,2);R(3,3). measurement noise covariance matrix
%X(2) -> Three values of inferior diag. of R -> R(4,4);R(5,5);R(6,6). measurement noise covariance matrix
%X(3) -> Three values of superior diag. of P -> P(1,1);P(2,2);P(3,3). error covariance matrix
%X(4) -> Three values of inferior diag. of P -> P(4,4);P(5,5);P(6,6). error covariance matrix
%X(5) -> gyro error noise
%X(6) -> gyro bias noise
R = 2*diag([0.143355206468984, 0.143355206468984, 0.143355206468984,1.043538860007335, 1.043538860007335, 1.043538860007335]); %measurement noise covariance matrix
P = 2*diag([0.019391168654884, 0.019391168654884, 0.019391168654884, 0.001636388033442, 0.001636388033442, 0.001636388033442]); %error covariance matrix
Gyro_errors_noise = 2*0.001635185410613;
Gyro_errors_bias = 2*1.445017964937501e-06;

%the test must start with a steady period
bw = mean(gyro_data(:,100:400),2);       %gyro bias
fn = mean(acc_data(:,100:400),2);          %gravity in the sensor frame
mn = mean(mag_data(:,100:400),2);       %magnetic field in the sensor frame

%align the board with the jig
q_bno = quatnormalize(Q');

%% comparing the results for the MINIMU -- FOR NOW WITH BNO'S MAGNETOMER 
%applying the fusion algorithms
q_CF = CF_iNemo(acc_data, gyro_data, mag_data, fs, bw,...
                            0.5, 0.5, .95, [-0.7874;-0.0259;-0.0198;0.6155]); q_CF = q_CF';
q_madgwick = madgwickAlgorithm(acc_data, gyro_data, mag_data, fs, ...
                            bw, 0.56,[-0.7887, -0.0261, -0.0199, 0.6139]);  %Quaternion = [-0.5911 0.0158 0.0068 -0.8064];
q_mahony = mahonyAlgorithm(acc_data, gyro_data, mag_data, fs, ...
                            bw, 0.5,[-0.7887, -0.0261, -0.0199, 0.6139]);  %Quaternion = [-0.5916 0.0156 0.0067 -0.8061]; 
[q_gyroLib, P, bw_] = Gyro_lib_quat(P, bw*(pi/180), gyro_data*(pi/180*(1/fs)), Gyro_errors_noise, Gyro_errors_bias,...
    acc_data, mag_data, fn, mn, 1/fs, R,[1, 0, 0, 0]);   
q_gyroLib(isnan(q_gyroLib(:,1)),1) = 1; %replace NaN by unit vector, usually the last measurement is NaN
q_gyroLib(isnan(q_gyroLib(:,2)),2:4) = 0;
q_gyroLib = quatconj(q_gyroLib); %the kalman gyrolib returns the quaternion conjugate
                        
%ajusting their references to the same as the jigs reference
%calculating the initial quaternion
q_zero_bno = avg_quaternion_markley(q_bno(avg_time,:));
q_zero_CF = avg_quaternion_markley(q_CF(avg_time,:));
q_zero_madgwick = avg_quaternion_markley(q_madgwick(avg_time,:));
q_zero_mahony = avg_quaternion_markley(q_mahony(avg_time,:));
q_zero_gyroLib = avg_quaternion_markley(q_gyroLib(avg_time,:));

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
 grid on; title('Fusion comparison'); ylabel('yaw'); %axis([0 inf -3 8]);
legend('BNO','Madgwick','Mahony','CF','Kalman','Orientation','Horizontal');
subplot(312);
plot(t,[e_bno(:,2) e_madgwick_imu(:,2) e_mahony_imu(:,2) e_CF_imu(:,2) e_gyroLib(:,2)]*180/pi);
grid on; ylabel('roll'); %axis([0 inf -3 3]);
subplot(313);
plot(t,[e_bno(:,3) e_madgwick_imu(:,3) e_mahony_imu(:,3) e_CF_imu(:,3) e_gyroLib(:,3)]*180/pi);
grid on; ylabel('pitch'); %axis([0 inf -3 3]);


%% nice plots for report
% filter performance
figure;
p = panel();
p.pack('v',3);
p(1).pack({1}, {100});
p(2).pack({1}, {100});
p(3).pack({1}, {100});
p.de.margin = 4;
p.margin = [15 15 5 10];
p.select('all');
p.fontsize = 10;
p.identify();

p(1,1,1).select();
plot(t, e_bno(:,1)*180/pi,'Linewidth',1.3); hold all; 
plot(t, e_madgwick_imu(:,1)*180/pi,':');
plot(t, e_mahony_imu(:,1)*180/pi,'-.','Linewidth',1.5);
plot(t, e_CF_imu(:,1)*180/pi);
plot(t, e_gyroLib(:,1)*180/pi,'--','Linewidth',1.5);
axis([0 inf -inf inf]); 
grid on; title('Fusion comparison'); ylabel('Yaw [°]');
legend('BNO','Madgwick','Mahony','CF','Kalman','Orientation','Horizontal');
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(t, e_bno(:,2)*180/pi,'Linewidth',1.3); hold all; 
plot(t, e_madgwick_imu(:,2)*180/pi,':');
plot(t, e_mahony_imu(:,2)*180/pi,'-.','Linewidth',1.5);
plot(t, e_CF_imu(:,2)*180/pi);
plot(t, e_gyroLib(:,2)*180/pi,'--','Linewidth',1.5);
axis([0 inf -inf inf]); 
grid on; ylabel('Roll [°]');
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(t, e_bno(:,3)*180/pi,'Linewidth',1.3); hold all; 
plot(t, e_madgwick_imu(:,3)*180/pi,':');
plot(t, e_mahony_imu(:,3)*180/pi,'-.','Linewidth',1.5);
plot(t, e_CF_imu(:,3)*180/pi);
plot(t, e_gyroLib(:,3)*180/pi,'--','Linewidth',1.5);
axis([0 inf -inf inf]); 
grid on; ylabel('Pitch [°]'); xlabel('Time [s]');
p(3).marginbottom = 10;


lis = normc(mag_imu_gaus);
bno_mag_norm = normc(mag_bno_gaus);

figure;
p = panel();
p.pack('v',3);
p(1).pack({1}, {100});
p(2).pack({1}, {100});
p(3).pack({1}, {100});
p.de.margin = 4;
p.margin = [15 15 5 5];
p.select('all');
p.fontsize = 10;
p.identify();

p(1,1,1).select();
plot(t,[lis(1,:)-mean(lis(1,:)); bno_mag_norm(1,:)-mean(bno_mag_norm(1,:))]'); grid on;
ylabel('X axis','FontSize',12);legend('LIS','BNO','Location','northeast','Orientation','horizontal')
axis([0 30 -inf inf])
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(t,[lis(2,:)-mean(lis(2,:)); bno_mag_norm(2,:)-mean(bno_mag_norm(2,:))]'); grid on;
ylabel('Y axis','FontSize',12);
axis([0 30 -inf inf])
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(t,[lis(3,:)-mean(lis(3,:)); bno_mag_norm(3,:)-mean(bno_mag_norm(3,:))]'); grid on;
ylabel('Z axis','FontSize',12);
axis([0 30 -inf inf])
xlabel('Time [s]','FontSize',12)
p(3).marginbottom = 10;