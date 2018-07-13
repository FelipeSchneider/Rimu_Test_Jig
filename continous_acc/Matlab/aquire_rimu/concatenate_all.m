addpath(genpath('..\fusion'));
addpath('..\');
clear all; clc; close all;
disp('Start:'); disp(datetime('now'));
%% concatenating
acc_bno = zeros(3,96*60000);
acc_imu = zeros(3,96*60000);
giro_bno = zeros(3,96*60000);
giro_imu = zeros(3,96*60000);
mag = zeros(3,96*60000);
temp = zeros(1,96*60000);
q_bno = zeros(4,96*60000);
for i=1:96
    name=sprintf('data_%d.mat',i);  % get the list of files
    load(name);
    acc_bno(:,(i-1)*60000+1:i*60000) = acc_bno_g;
    acc_imu(:,(i-1)*60000+1:i*60000) = acc_imu_g;
    giro_bno(:,(i-1)*60000+1:i*60000) = giro_bno_dps;
    giro_imu(:,(i-1)*60000+1:i*60000) = giro_imu_dps;
    mag(:,(i-1)*60000+1:i*60000) = mag_imu_gaus;
    temp(1,(i-1)*60000+1:i*60000) = temperature_C;
    q_bno(:,(i-1)*60000+1:i*60000) = Q;
end
q_bno = q_bno';
load open_space_mag_cal.mat
mag_imu_gaus_cal = mag' * Ca_imu' + repmat(Cb_imu', length(mag), 1);
mag_imu_gaus_cal = mag_imu_gaus_cal';

%% Data for DFM
Rot_Order = 'ZXY';
acc_data = (acc_imu + acc_bno)/2;
gyro_data = (giro_imu + giro_bno)/2;
mag_data = mag_imu_gaus_cal;
fs = 100;

R = diag([0.106081894043137, 0.106081894043137, 0.106081894043137,0.546630894966456, 0.546630894966456, 0.546630894966456])*2; %measurement noise covariance matrix
P = diag([0.0286839110161009, 0.0286839110161009, 0.0286839110161009, 0.000350603145874234, 0.000350603145874234, 0.000350603145874234]); %error covariance matrix
Gyro_errors_noise = 0.000792992205142529;
Gyro_errors_bias = 8.54935619884057e-07;
Kp = 1.63;          %Mahony
Beta = 0.155;       %Madgwick 
Crossover = 0.5;  %CF

%the test must start with a steady period
bw = mean(gyro_data(:,100:400),2);       %gyro bias
fn = mean(acc_data(:,100:400),2);          %gravity in the sensor frame
mn = mean(mag_data(:,100:400),2);       %magnetic field in the sensor frame

%% time calculation
t = (1:length(temp))/(100*60*60);
temp = movmedian(temp, 7);

%% Raw data plots
figure(13);
plot(t',temp');
title('Temperature (°C)'); xlabel('Time [hour]');

figure(10);
subplot(311);plot(t,[giro_imu(1,:)-mean(giro_imu(1,100:400),2); giro_bno(1,:)-mean(giro_bno(1,100:400),2)]'); grid on;
title('Giroscópio');ylabel('X [dps]');legend('Minimu','BNO','Location','northwest','Orientation','horizontal')
subplot(312);plot(t,[giro_imu(2,:)-mean(giro_imu(2,100:400),2); giro_bno(2,:)-mean(giro_bno(2,100:400),2)]'); ylabel('Y [dps]'); grid on;
subplot(313);plot(t,[giro_imu(3,:)-mean(giro_imu(3,100:400),2); giro_bno(3,:)-mean(giro_bno(3,100:400),2)]'); ylabel('Z [dps]'); grid on;
xlabel('Tempo [s]')

figure(11);
subplot(311);plot(t,[acc_imu(1,:); acc_bno(1,:)]'); grid on;
title('Acelerômetro');ylabel('X [g]');legend('Minimu','BNO','Location','northwest','Orientation','horizontal')
subplot(312);plot(t,[acc_imu(2,:); acc_bno(2,:)]'); ylabel('Y [g]'); grid on;
subplot(313);plot(t,[acc_imu(3,:); acc_bno(3,:)]'); ylabel('Z [g]'); grid on;
xlabel('Tempo [s]')

figure(12);
subplot(311);plot(t,mag_imu_gaus_cal(1,:)'); grid on; grid on;
title('Magnetômetro');ylabel('X');legend('Minimu','Location','northwest','Orientation','horizontal')
subplot(312);plot(t,mag_imu_gaus_cal(2,:)'); ylabel('Y'); grid on;
subplot(313);plot(t,mag_imu_gaus_cal(3,:)'); ylabel('Z'); grid on;
xlabel('Tempo [s]')
disp('Start DFMs'); disp(datetime('now'));

%% comparing the results
%applying the fusion algorithms
q_madgwick = madgwickAlgorithm(acc_data, gyro_data, mag_data, fs, ...
                            bw, Beta);  %Quaternion = [-0.5911 0.0158 0.0068 -0.8064];
disp('Madgwick'); disp(datetime('now'));
q_mahony = mahonyAlgorithm(acc_data, gyro_data, mag_data, fs, ...
                            bw, Kp);  %Quaternion = [-0.5916 0.0156 0.0067 -0.8061]; 
disp('Mahony'); disp(datetime('now'));
[q_gyroLib, P, bw_] = Gyro_lib_quat(P, bw*(pi/180), gyro_data*(pi/180*(1/fs)), Gyro_errors_noise, Gyro_errors_bias,...
    acc_data, mag_data, fn, mn, 1/fs, R);   
disp('Kalman'); disp(datetime('now'));
q_gyroLib(isnan(q_gyroLib(:,1)),1) = 1; %replace NaN by unit vector, usually the last measurement is NaN
q_gyroLib(isnan(q_gyroLib(:,2)),2:4) = 0;
q_gyroLib = quatconj(q_gyroLib); %the kalman gyrolib returns the quaternion conjugate

q_CF = CF_iNemo(acc_data, gyro_data, mag_data, fs, bw,...
                            0.5, 0.5, Crossover, [0.303169; 0.3276608; 0.62060; 0.64465]); q_CF = q_CF';
disp('CF'); disp(datetime('now'));                        
%ajusting their references to the same as the jigs reference
%calculating the initial quaternion
q_zero_bno = avg_quaternion_markley(q_bno(4500:5000,:));
q_zero_CF = avg_quaternion_markley(q_CF(4500:5000,:));
q_zero_madgwick = avg_quaternion_markley(q_madgwick(4500:5000,:));
q_zero_mahony = avg_quaternion_markley(q_mahony(4500:5000,:));
q_zero_gyroLib = avg_quaternion_markley(q_gyroLib(4500:5000,:));

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

temp = movmedian(temp, 7);
e_bno = movmedian(e_bno, 7);
e_madgwick_imu = movmedian(e_madgwick_imu, 7);
e_mahony_imu = movmedian(e_mahony_imu, 7);
e_CF_imu = movmedian(e_CF_imu, 7);
e_gyroLib = movmedian(e_gyroLib, 7);
%% Plots
figure;
subplot(311);
plot(t,[e_madgwick_imu(:,1) e_mahony_imu(:,1) e_bno(:,1) e_CF_imu(:,1)]*180/pi);
axis([0 inf -3 3]); grid on; title('Fusion comparison'); ylabel('yaw');
legend('Madgwick','Mahony','BNO','CF','Orientation','Horizontal');
subplot(312);
plot(t,[e_madgwick_imu(:,2) e_mahony_imu(:,2) e_bno(:,2) e_CF_imu(:,2)]*180/pi);
axis([0 inf -3 3]); grid on; ylabel('roll');
subplot(313);
plot(t,[e_madgwick_imu(:,3) e_mahony_imu(:,3) e_bno(:,3) e_CF_imu(:,3)]*180/pi);
axis([0 inf -10 10]); grid on; ylabel('pitch');

%original data figure
figure;
p = panel();
p.pack('v',4);
p(1).pack({1}, {100});
p(2).pack({1}, {100});
p(3).pack({1}, {100});
p(4).pack({1}, {100});
p.de.margin = 4;
p.margin = [15 15 5 10];
p.select('all');
p.fontsize = 10;
p.identify();

p(1,1,1).select();
plot(t, e_bno(:,1)*180/pi,':','Linewidth',1.3); hold all;
plot(t, e_madgwick_imu(:,1)*180/pi);
plot(t, e_mahony_imu(:,1)*180/pi,'-.','Linewidth',1.5);
plot(t, e_CF_imu(:,1)*180/pi);
plot(t, e_gyroLib(:,1)*180/pi,'--','Linewidth',1.5);
axis([0 inf -1 1]);
grid on; ylabel('Yaw error [°]');
legend('BNO','Madgwick','Mahony','CF','Kalman','Orientation','Horizontal');
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(t, e_bno(:,2)*180/pi,':','Linewidth',1.3); hold all;
plot(t, e_madgwick_imu(:,2)*180/pi);
plot(t, e_mahony_imu(:,2)*180/pi,'-.','Linewidth',1.5);
plot(t, e_CF_imu(:,2)*180/pi);
plot(t, e_gyroLib(:,2)*180/pi,'--','Linewidth',1.5);
axis([0 inf -2 2]); grid on; ylabel('Roll error [°]');
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(t, e_bno(:,3)*180/pi,':','Linewidth',1.3); hold all;
plot(t, e_madgwick_imu(:,3)*180/pi);
plot(t, e_mahony_imu(:,3)*180/pi,'-.','Linewidth',1.5);
plot(t, e_CF_imu(:,3)*180/pi);
plot(t, e_gyroLib(:,3)*180/pi,'--','Linewidth',1.5);
axis([0 inf -10 10]); grid on; 
ylabel('Pitch error [°]');  a = gca; a.XTickLabel = {};

p(4,1,1).select();
plot(t, temp,'Linewidth',1.3); hold all;
axis([0 inf -inf inf]); grid on; ylabel('Temperature [°C]');
xlabel('Time [h]');
p(4).marginbottom = 10;

%filtered data figure
e_bno2 = movmean(e_bno, 1000);
e_madgwick_imu2 = movmean(e_madgwick_imu, 1000);
e_mahony_imu2 = movmean(e_mahony_imu, 1000);
e_CF_imu2 = movmean(e_CF_imu, 1000);
e_gyroLib2 = movmean(e_gyroLib, 1000);

cor = lines(5);

figure;
p = panel();
p.pack('v',4);
p(1).pack({1}, {100});
p(2).pack({1}, {100});
p(3).pack({1}, {100});
p(4).pack({1}, {100});
p.de.margin = 4;
p.margin = [15 15 5 10];
p.select('all');
p.fontsize = 10;
p.identify();

p(1,1,1).select();
plot(t, e_gyroLib2(:,1)*180/pi,'Color',cor(5,:)); hold all;
plot(t, e_CF_imu2(:,1)*180/pi,'Linewidth',1.3,'Color',cor(4,:));
plot(t, e_mahony_imu2(:,1)*180/pi,'Color',cor(3,:));
plot(t, e_madgwick_imu2(:,1)*180/pi,'Color',cor(2,:));
plot(t, e_bno2(:,1)*180/pi,'Linewidth',1.3,'Color',cor(1,:));
axis([0 inf -0.5 0.5]);
grid on; ylabel('Yaw error [°]');
legend('Kalman','CF','Mahony','Madgwick','BNO','Orientation','Horizontal');
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(t, e_gyroLib2(:,2)*180/pi,'Color',cor(5,:)); hold all;
plot(t, e_CF_imu2(:,2)*180/pi,'Linewidth',1.3,'Color',cor(4,:));
plot(t, e_mahony_imu2(:,2)*180/pi,'Color',cor(3,:));
plot(t, e_madgwick_imu2(:,2)*180/pi,'Color',cor(2,:));
plot(t, e_bno2(:,2)*180/pi,'Linewidth',1.3,'Color',cor(1,:));
axis([0 inf -1 1]); grid on; ylabel('Roll error [°]');
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(t, e_gyroLib2(:,3)*180/pi,'Color',cor(5,:)); hold all;
plot(t, e_CF_imu2(:,3)*180/pi,'Linewidth',1.3,'Color',cor(4,:));
plot(t, e_mahony_imu2(:,3)*180/pi,'Color',cor(3,:));
plot(t, e_madgwick_imu2(:,3)*180/pi,'Color',cor(2,:));
plot(t, e_bno2(:,3)*180/pi,'Linewidth',1.3,'Color',cor(1,:));
axis([0 inf -7 10]); grid on; 
ylabel('Pitch error [°]');  a = gca; a.XTickLabel = {};

p(4,1,1).select();
plot(t, temp,'Linewidth',1.3); hold all;
axis([0 inf -inf inf]); grid on; ylabel('Temperature [°C]');
xlabel('Time [h]');
p(4).marginbottom = 10;

%original gyro data
figure;
p = panel();
p.pack('v',4);
p(1).pack({1}, {100});
p(2).pack({1}, {100});
p(3).pack({1}, {100});
p(4).pack({1}, {100});
p.de.margin = 4;
p.margin = [15 15 5 10];
p.select('all');
p.fontsize = 10;
p.identify();

p(1,1,1).select();
plot(t, giro_imu(1,:)-mean(giro_imu(1,100:400),2)); hold all;
plot(t, giro_bno(1,:)-mean(giro_bno(1,100:400),2));
axis([0 inf -inf inf]);
grid on; ylabel('X axis [dps]');
legend('LSM','BNO','Orientation','Horizontal');
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(t, giro_imu(2,:)-mean(giro_imu(2,100:400),2)); hold all;
plot(t, giro_bno(2,:)-mean(giro_bno(2,100:400),2));
axis([0 inf -inf inf]); grid on; ylabel('Y axis [dps]');
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(t, giro_imu(3,:)-mean(giro_imu(3,100:400),2)); hold all;
plot(t, giro_bno(3,:)-mean(giro_bno(2,100:400),2));
axis([0 inf -inf inf]); grid on; 
ylabel('Z axis [dps]');  a = gca; a.XTickLabel = {};

p(4,1,1).select();
plot(t, temp,'Linewidth',1.3); hold all;
axis([0 inf -inf inf]); grid on; ylabel('Temperature [°C]');
xlabel('Time [h]');
p(4).marginbottom = 10;


%original acc data
figure;
p = panel();
p.pack('v',4);
p(1).pack({1}, {100});
p(2).pack({1}, {100});
p(3).pack({1}, {100});
p(4).pack({1}, {100});
p.de.margin = 4;
p.margin = [15 15 5 10];
p.select('all');
p.fontsize = 10;
p.identify();

p(1,1,1).select();
plot(t, acc_imu(1,:)); hold all;
plot(t, acc_bno(1,:));
axis([0 inf -inf inf]);
grid on; ylabel('X axis [g]');
legend('LSM','BNO','Orientation','Horizontal');
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(t, acc_imu(2,:)); hold all;
plot(t, acc_bno(2,:));
axis([0 inf -inf inf]); grid on; ylabel('Y axis [g]');
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(t, acc_imu(3,:)); hold all;
plot(t, acc_bno(3,:));
axis([0 inf -inf inf]); grid on; 
ylabel('Z axis [g]');  a = gca; a.XTickLabel = {};

p(4,1,1).select();
plot(t, temp,'Linewidth',1.3); hold all;
axis([0 inf -inf inf]); grid on; ylabel('Temperature [°C]');
xlabel('Time [h]');
p(4).marginbottom = 10;


