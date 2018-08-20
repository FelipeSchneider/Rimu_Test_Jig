addpath('C:\Users\felip\Dropbox\Mestrado\Dissertação\Coletas Jiga\Teste_giro_filt')  %just to be able to load previously jig measurements
addpath(genpath('C:\Users\felip\Documents\Arquivos dissertação\Testes dissertação'))
addpath(genpath('..\fusion'));
addpath('..\');
clear all;
load Open_space_cal_mat.mat
%load 5cm-trafo.mat
%load 5cm.mat
load open_space_90_degrees.mat
cor = lines(6);
%% calibrate LSM gyro data
giro_imu_dps = giro_imu_dps - mean(giro_imu_dps(:,100:400),2);
giro_imu_dps = giro_imu_dps*1.1692104;

%% calibrate LIS data
%[mag_imu_gaus_cal, Ca_imu, Cb_imu] = magCalibration(mag_imu_gaus');
mag_imu_gaus_cal = mag_imu_gaus' * Ca_imu' + repmat(Cb_imu', length(mag_imu_gaus), 1);
mag_imu_gaus_cal = mag_imu_gaus_cal';
%% Data that must be pre entered
Rot_Order = 'ZXY';
acc_data = (acc_imu_g + acc_bno_g)/2;
gyro_data = (giro_imu_dps + giro_bno_dps)/2;
mag_data = (mag_imu_gaus_cal + mag_bno_gaus)/2;
avg_time = 500:700;
%Kalman constants
%X(1) -> Three values of superior diag. of R -> R(1,1);R(2,2);R(3,3). measurement noise covariance matrix
%X(2) -> Three values of inferior diag. of R -> R(4,4);R(5,5);R(6,6). measurement noise covariance matrix
%X(3) -> Three values of superior diag. of P -> P(1,1);P(2,2);P(3,3). error covariance matrix
%X(4) -> Three values of inferior diag. of P -> P(4,4);P(5,5);P(6,6). error covariance matrix
%X(5) -> gyro error noise
%X(6) -> gyro bias noise
%Slow-range test
% X_best = [0.048178218974686;0.109923250960862;0.005811115831348;0.007798193953625;0.006819123339871;1.007453221846325e-06];  %mid range
% Kp = 2.630;          %Mahony 
% Beta = 0.210;       %Madgwick 
% Crossover = 0.977;  %CF

%Mid-range test
X_best = [0.0586601347080640;0.757093853961796;0.00517511309968247;0.00946844040862660;0.000956036619570417;1.27615921715542e-06];  %mid range
Kp = 1.720;          %Mahony 1.630
Beta = 0.100;       %Madgwick 0.155
Crossover = 0.987;  %CF

R = diag([X_best(1), X_best(1), X_best(1), X_best(2), X_best(2), X_best(2)]); %measurement noise covariance matrix
P = diag([X_best(3), X_best(3), X_best(3), X_best(4), X_best(4), X_best(4)]); %error covariance matrix
Gyro_errors_noise = X_best(5);
Gyro_errors_bias = X_best(6);

%the test must start with a steady period
bw = mean(gyro_data(:,100:400),2);       %gyro bias
fn = mean(acc_data(:,100:400),2);          %gravity in the sensor frame
mn = mean(mag_data(:,100:400),2);       %magnetic field in the sensor frame

%align the board with the jig
q_bno = quatnormalize(Q');

%% comparing the results for the MINIMU -- FOR NOW WITH BNO'S MAGNETOMER 
%applying the fusion algorithms
q_CF = CF_iNemo(acc_data, gyro_data, mag_data, fs, bw,...
                            0.5, 0.5, Crossover, [-0.7874;-0.0259;-0.0198;0.6155]); q_CF = q_CF';
q_madgwick = madgwickAlgorithm(acc_data, gyro_data, mag_data, fs, ...
                            bw, Beta,[-0.7887, -0.0261, -0.0199, 0.6139]);  %Quaternion = [-0.5911 0.0158 0.0068 -0.8064];
q_mahony = mahonyAlgorithm(acc_data, gyro_data, mag_data, fs, ...
                            bw, 2.630,[-0.7887, -0.0261, -0.0199, 0.6139]);  %Quaternion = [-0.5916 0.0156 0.0067 -0.8061]; 
[q_gyroLib, P, bw_] = Gyro_lib_quat(P, bw*(pi/180), gyro_data*(pi/180*(1/fs)), Gyro_errors_noise, Gyro_errors_bias,...
    acc_data, mag_data, fn, mn, 1/fs, R,[1, 0, 0, 0]);  


%for the mag-test
% q_CF = CF_iNemo(acc_data, gyro_data, mag_data, fs, bw,...
%                             0.5, 0.5, Crossover, [-0.647375632753986;0.007845585464126;-4.591949391835755e-04;-0.762130273884649]); q_CF = q_CF';
% q_madgwick = madgwickAlgorithm(acc_data, gyro_data, mag_data, fs, ...
%                             bw, Beta,[0.648534959592992 -0.007766459751053 6.502279072075808e-04 0.761141240600150]);  %Quaternion = [-0.5911 0.0158 0.0068 -0.8064];
% q_mahony = mahonyAlgorithm(acc_data, gyro_data, mag_data, fs, ...
%                             bw, 2.630,[-0.648283594023949 0.007721679923759 -4.512090276622195e-04 -0.761359427942524]);  %Quaternion = [-0.5916 0.0156 0.0067 -0.8061]; 
% [q_gyroLib, P, bw_] = Gyro_lib_quat(P, bw*(pi/180), gyro_data*(pi/180*(1/fs)), Gyro_errors_noise, Gyro_errors_bias,...
%     acc_data, mag_data, fn, mn, 1/fs, R,[1, 0, 0, 0]);

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
[e_madgwick(:,1), e_madgwick(:,2), e_madgwick(:,3)] = quat2angle(q_madgwick, Rot_Order);
[e_mahony(:,1), e_mahony(:,2), e_mahony(:,3)] = quat2angle(q_mahony, Rot_Order);
[e_CF(:,1), e_CF(:,2), e_CF(:,3)] = quat2angle(q_CF, Rot_Order);
[e_gyroLib(:,1), e_gyroLib(:,2), e_gyroLib(:,3)] = quat2angle(q_gyroLib, Rot_Order);

%% quasi-static reference
ref = NaN(length(acc_data),1);
ref(10*fs:30*fs) = 0;
ref(50*fs:70*fs) = -10;
ref(90*fs:110*fs) = -20;
ref(125*fs:145*fs) = -30;
ref(170*fs:190*fs) = -40;
ref(210*fs:230*fs) = -50;
ref(250*fs:270*fs) = -60;
ref(295*fs:315*fs) = -70;
ref(335*fs:355*fs) = -80;
ref(379*fs:399*fs) = -90;

e_diff_bno = angdiff(ref*pi/180,e_bno(:,1));
e_diff_madgwick = angdiff(ref*pi/180,e_madgwick(:,1));
e_diff_mahony = angdiff(ref*pi/180,e_mahony(:,1));
e_diff_CF = angdiff(ref*pi/180,e_CF(:,1));
e_diff_gyroLib = angdiff(ref*pi/180,e_gyroLib(:,1));

figure;
plot(t, e_diff_bno(:,1)*180/pi,':','Linewidth',1.3,'color',cor(2,:)); hold all;
plot(t, e_diff_madgwick(:,1)*180/pi,'color',cor(3,:));
plot(t, e_diff_mahony(:,1)*180/pi,'-.','Linewidth',1.5,'color',cor(4,:));
plot(t, e_diff_CF(:,1)*180/pi,'color',cor(5,:));
plot(t, e_diff_gyroLib(:,1)*180/pi,'--','Linewidth',1.5,'color',cor(6,:));
axis([0 inf -4 4]);
grid on; title('Fusion comparison'); 
ylabel('Yaw error [°]'); xlabel('Time [s]');
legend('BNO','Madgwick','Mahony','CF','Kalman','Orientation','Horizontal');
text(20,-1,'0°','HorizontalAlignment','center','FontSize',11,'FontWeight','bold')
text(60,-1,'-10°','HorizontalAlignment','center','FontSize',11,'FontWeight','bold')
text(100,-0.5,'-20°','HorizontalAlignment','center','FontSize',11,'FontWeight','bold')
text(135,0,'-30°','HorizontalAlignment','center','FontSize',11,'FontWeight','bold')
text(180,-1.5,'-40°','HorizontalAlignment','center','FontSize',11,'FontWeight','bold')
text(220,-0.5,'-50°','HorizontalAlignment','center','FontSize',11,'FontWeight','bold')
text(260,-0.5,'-60°','HorizontalAlignment','center','FontSize',11,'FontWeight','bold')
text(305,0,'-70°','HorizontalAlignment','center','FontSize',11,'FontWeight','bold')
text(345,0,'-80°','HorizontalAlignment','center','FontSize',11,'FontWeight','bold')
text(390,-2.5,'-90°','HorizontalAlignment','center','FontSize',11,'FontWeight','bold')

e_diff_bno(isnan(e_diff_bno)) = [];
e_diff_madgwick(isnan(e_diff_madgwick)) = [];
e_diff_mahony(isnan(e_diff_mahony)) = [];
e_diff_CF(isnan(e_diff_CF)) = [];
e_diff_gyroLib(isnan(e_diff_gyroLib)) = [];

kte_bno = sqrt(mean(abs(e_diff_bno*180/pi)).^2 + var(e_diff_bno*180/pi))';
kte_madgwick = sqrt(mean(abs(e_diff_madgwick*180/pi)).^2 + var(e_diff_madgwick*180/pi))';
kte_mahony = sqrt(mean(abs(e_diff_mahony*180/pi)).^2 + var(e_diff_mahony*180/pi))';
kte_CF = sqrt(mean(abs(e_diff_CF*180/pi)).^2 + var(e_diff_CF*180/pi))';
kte_gyroLib = sqrt(mean(abs(e_diff_gyroLib*180/pi)).^2 + var(e_diff_gyroLib*180/pi))';

rms_bno = rms(e_diff_bno*180/pi)';
rms_madgwick = rms(e_diff_madgwick*180/pi)';
rms_mahony = rms(e_diff_mahony*180/pi)';
rms_CF = rms(e_diff_CF*180/pi)';
rms_gyroLib = rms(e_diff_gyroLib*180/pi)';

std_bno = sqrt(var(e_diff_bno(:,1)*180/pi));
std_madgwick = sqrt(var(e_diff_madgwick(:,1)*180/pi));
std_mahony = sqrt(var(e_diff_mahony(:,1)*180/pi));
std_CF = sqrt(var(e_diff_CF(:,1)*180/pi));
std_gyroLib = sqrt(var(e_diff_gyroLib(:,1)*180/pi));
%% Plots
figure;
subplot(311);
plot(t,[e_bno(:,1) e_madgwick(:,1) e_mahony(:,1) e_CF(:,1) e_gyroLib(:,1)]*180/pi);
grid on; title('Fusion comparison'); ylabel('yaw'); %axis([0 inf -3 8]);
legend('BNO','Madgwick','Mahony','CF','Kalman','Orientation','Horizontal');
subplot(312);
plot(t,[e_bno(:,2) e_madgwick(:,2) e_mahony(:,2) e_CF(:,2) e_gyroLib(:,2)]*180/pi);
grid on; ylabel('roll'); %axis([0 inf -3 3]);
subplot(313);
plot(t,[e_bno(:,3) e_madgwick(:,3) e_mahony(:,3) e_CF(:,3) e_gyroLib(:,3)]*180/pi);
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
plot(t, e_bno(:,1)*180/pi,':','Linewidth',1.3,'color',cor(2,:)) ; hold all;
plot(t, e_madgwick(:,1)*180/pi,'color',cor(3,:));
plot(t, e_mahony(:,1)*180/pi,'-.','Linewidth',1.5,'color',cor(4,:));
plot(t, e_CF(:,1)*180/pi,'color',cor(5,:));
plot(t, e_gyroLib(:,1)*180/pi,'--','Linewidth',1.5,'color',cor(6,:));
%hold on; plot(t,ref,'r','Linewidth',2);
axis([0 inf -inf inf]); 
grid on; title('Fusion comparison'); ylabel('Yaw [°]');
legend('BNO','Madgwick','Mahony','CF','Kalman','Ref.','Orientation','Horizontal');
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(t, e_bno(:,2)*180/pi,':','Linewidth',1.3,'color',cor(2,:)) ;hold all;
plot(t, e_madgwick(:,2)*180/pi,'color',cor(3,:));
plot(t, e_mahony(:,2)*180/pi,'-.','Linewidth',1.5,'color',cor(4,:));
plot(t, e_CF(:,2)*180/pi,'color',cor(5,:));
plot(t, e_gyroLib(:,2)*180/pi,'--','Linewidth',1.5,'color',cor(6,:));
axis([0 inf -inf inf]); 
grid on; ylabel('Roll [°]');
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(t, e_bno(:,3)*180/pi,':','Linewidth',1.3,'color',cor(2,:)) ;hold all;
plot(t, e_madgwick(:,3)*180/pi,'color',cor(3,:));
plot(t, e_mahony(:,3)*180/pi,'-.','Linewidth',1.5,'color',cor(4,:));
plot(t, e_CF(:,3)*180/pi,'color',cor(5,:));
plot(t, e_gyroLib(:,3)*180/pi,'--','Linewidth',1.5,'color',cor(6,:));
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

%mag performance
dev_CF = max(abs(e_CF*180/pi))';
dev_gyroLib = max(abs(e_gyroLib*180/pi))';
dev_mahony = max(abs(e_mahony*180/pi))';
dev_madgwick = max(abs(e_madgwick*180/pi))';