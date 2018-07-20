%addpath('C:\Users\felip\Dropbox\Mestrado\Dissertação\Coletas Jiga\Teste_giro_filt')  %just to be able to load previously jig measurements
%addpath(genpath('C:\Users\felip\Documents\Arquivos dissertação\Testes dissertação\data colection'))
addpath(genpath('C:\Users\felip\Documents\Arquivos dissertação\Testes dissertação\_new data collection'))
addpath(genpath('fusion'));
addpath('jig');
%addpath('aquire_rimu');

%load yaw_comp_data.mat
%load pitch_comp_data.mat
%load pitch_comp_data.mat
%load slow_range_data.mat
%load slow_range_data2.mat
load mid_range_data.mat
%load fast_range_data.mat

clearvars -except acc_bno_g acc_imu_g com_data description fs giro_bno_dps giro_imu_dps ...
    jig_const mag_bno_gaus mag_imu_gaus Q real_base_angle real_top_angle t...
    t_angles t_imu t_rec

load LIS_cal_mat
load Yaw_alignment
load Pitch_alignment

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
%mag_data = mag_imu_gaus_cal;
mag_data = mag_bno_gaus;
avg_time_quat = 250:350;
avg_time_sens = 100:350;
%Kalman constants
%X(1) -> Three values of superior diag. of R -> R(1,1);R(2,2);R(3,3). measurement noise covariance matrix
%X(2) -> Three values of inferior diag. of R -> R(4,4);R(5,5);R(6,6). measurement noise covariance matrix
%X(3) -> Three values of superior diag. of P -> P(1,1);P(2,2);P(3,3). error covariance matrix
%X(4) -> Three values of inferior diag. of P -> P(4,4);P(5,5);P(6,6). error covariance matrix
%X(5) -> gyro error noise
%X(6) -> gyro bias noise
R = diag([0.143355206468984, 0.143355206468984, 0.143355206468984,1.043538860007335, 1.043538860007335, 1.043538860007335]); %measurement noise covariance matrix
P = diag([0.019391168654884, 0.019391168654884, 0.019391168654884, 0.001636388033442, 0.001636388033442, 0.001636388033442]); %error covariance matrix
Gyro_errors_noise = 0.001635185410613;
Gyro_errors_bias = 1.445017964937501e-06;
Kp = 1.630;          %Mahony 1.630
Beta = 0.155;       %Madgwick 0.155
Crossover = 0.883;  %CF

%the test must start with a steady period
bw = mean(gyro_data(:,avg_time_sens),2);       %gyro bias
fn = mean(acc_data(:,avg_time_sens),2);          %gravity in the sensor frame
mn = mean(mag_data(:,avg_time_sens),2);       %magnetic field in the sensor frame

%% comparing the results for the RIMU
%applying the fusion algorithms
disp('Starting with fusion methods');
q_CF = CF_iNemo(acc_data, gyro_data, mag_data, fs, bw,...
                            0.5, 0.5, Crossover, [-0.8010; 0.0010; -0.0172; 0.5984]); q_CF = q_CF';
q_madgwick = madgwickAlgorithm(acc_data, gyro_data, mag_data, fs, ...
                            bw, Beta,[-0.8009, 0.0005, -0.0177, 0.5985]);
q_mahony = mahonyAlgorithm(acc_data, gyro_data, mag_data, fs, ...
                            bw, Kp,[-0.8059, -0.0012, -0.0189,  0.5918]);
[q_gyroLib, P, bw_] = Gyro_lib_quat(P, bw*(pi/180), gyro_data*(pi/180*(1/fs)), Gyro_errors_noise, Gyro_errors_bias,...
    acc_data, mag_data, fn, mn, 1/fs, R, [-1.0000,-0.0002, 0.0001, 0.0020]);   
q_gyroLib(isnan(q_gyroLib(:,1)),1) = 1; %replace NaN by unit vector, usually the last measurement is NaN
q_gyroLib(isnan(q_gyroLib(:,2)),2:4) = 0;
q_gyroLib = quatconj(q_gyroLib); %the kalman gyrolib returns the quaternion conjugate

disp('Rotating initial orientation');                        
%ajusting their references to the same as the jigs reference
%calculating the initial quaternion
q_bno = quatnormalize(Q');
q_zero_bno = avg_quaternion_markley(q_bno(avg_time_quat,:));
q_zero_CF = avg_quaternion_markley(q_CF(avg_time_quat,:));
q_zero_madgwick = avg_quaternion_markley(q_madgwick(avg_time_quat,:));
q_zero_mahony = avg_quaternion_markley(q_mahony(avg_time_quat,:));
q_zero_gyroLib = avg_quaternion_markley(q_gyroLib(avg_time_quat,:));

%subtracting the zero position from the quaternions
q_bno = quatmultiply(quatconj(q_zero_bno'), q_bno); q_bno = quatnormalize(q_bno);
q_CF = quatmultiply(quatconj(q_zero_CF'),q_CF);
q_madgwick = quatmultiply(quatconj(q_zero_madgwick'),q_madgwick);
q_mahony = quatmultiply(quatconj(q_zero_mahony'),q_mahony);
q_gyroLib = quatmultiply(quatconj(q_zero_gyroLib'),q_gyroLib);

%converting from quaternions to euler
[e_bno(:,1),e_bno(:,2),e_bno(:,3)] = quat2angle(q_bno(1:length(q_CF),:), Rot_Order);
[e_madgwick(:,1), e_madgwick(:,2), e_madgwick(:,3)] = quat2angle(q_madgwick, Rot_Order);
[e_mahony(:,1), e_mahony(:,2), e_mahony(:,3)] = quat2angle(q_mahony, Rot_Order);
[e_CF(:,1), e_CF(:,2), e_CF(:,3)] = quat2angle(q_CF, Rot_Order);
[e_gyroLib(:,1), e_gyroLib(:,2), e_gyroLib(:,3)] = quat2angle(q_gyroLib, Rot_Order);

%% Jig time ajustment
t_rec(1) = [];                  %the first position is a zero, used only for prealocation
t_expect = com_data(1,:);       %extracting the expect time for each command
t_expect(end+1) = t(end);       %filling with the end of all comands here
%t = t-t_rec(1);
%comp_factor = 1;
comp_factor = (t_rec(end))/(t_expect(end));
%comp_factor = 123.5/123.0383;    %took from graph FUSION4
%comp_factor = 56.7/56.8678;      %took from graph FUSION5
%comp_factor = 254.13/253.07818;  %FUSION 6
t_jig_comp = t*comp_factor;

%just to plot the marks on the graph
b_angle_expect = com_data(2,:); b_angle_expect(end+1) = real_base_angle(end);
t_angle_expect = com_data(3,:); t_angle_expect(end+1) = real_top_angle(end);

%wrap the aligned angles around -180 and 180 degrees
b_angle = wrapTo180(real_base_angle); b_angle_expect = wrapTo180(b_angle_expect);
t_angle = wrapTo180(real_top_angle);  t_angle_expect = wrapTo180(t_angle_expect);
%e_bno(end,:) = []; b_angle(end) = [];  t_angle(end) = [];

%match the imu and jig times
[t_match, base_match, top_match] = matchJigIMU(t_jig_comp, t_imu, b_angle, t_angle);
%base_match = -base_match;
q_jig = zeros(length(t_match),4);
for i=1:length(top_match)
    q_jig(i,:) = angle2quat(base_match(i)*pi/180, 0, -top_match(i)*pi/180, Rot_Order);
end
[e_jig(:,1),e_jig(:,2),e_jig(:,3)] = quat2angle(q_jig, Rot_Order);

[ e_bno, e_madgwick, e_mahony, e_CF, e_gyroLib ] ...
    = jigTimeAlign( e_bno, e_madgwick, e_mahony, e_CF, e_gyroLib, e_jig );

%% Jig angle correction
%correction for the yaw 

if(exist('pol_yaw','var'))
    disp('Aligning Yaw'); 
    yaw_comp = (e_jig(:,1).^4*pol_yaw.p1 + e_jig(:,1).^3*pol_yaw.p2 + e_jig(:,1).^2*pol_yaw.p3 + e_jig(:,1)*pol_yaw.p4 + pol_yaw.p5);
    roll_comp = (e_jig(:,1).^4*pol_roll.p1 + e_jig(:,1).^3*pol_roll.p2 + e_jig(:,1).^2*pol_roll.p3 + e_jig(:,1)*pol_roll.p4 + pol_roll.p5);
    pitch_comp = (e_jig(:,1).^4*pol_pitch.p1 + e_jig(:,1).^3*pol_pitch.p2 + e_jig(:,1).^2*pol_pitch.p3 + e_jig(:,1)*pol_pitch.p4 + pol_pitch.p5);

    e_bno(:,1) = e_bno(:,1)+yaw_comp;  
    e_bno(:,2) = e_bno(:,2)+roll_comp;  
    e_bno(:,3) = e_bno(:,3)+pitch_comp;

    e_madgwick(:,1) = e_madgwick(:,1)+yaw_comp; 
    e_madgwick(:,2) = e_madgwick(:,2)+roll_comp;  
    e_madgwick(:,3) = e_madgwick(:,3)+pitch_comp;

    e_mahony(:,1) = e_mahony(:,1)+yaw_comp;  
    e_mahony(:,2) = e_mahony(:,2)+roll_comp;  
    e_mahony(:,3) = e_mahony(:,3)+pitch_comp;

    e_CF(:,1) = e_CF(:,1)+yaw_comp;  
    e_CF(:,2) = e_CF(:,2)+roll_comp; 
    e_CF(:,3) = e_CF(:,3)+pitch_comp;

    e_gyroLib(:,1) = e_gyroLib(:,1)+yaw_comp;  
    e_gyroLib(:,2) = e_gyroLib(:,2)+roll_comp;  
    e_gyroLib(:,3) = e_gyroLib(:,3)+pitch_comp;
end

%correction for the pitch 
if(exist('pitch_pol_yaw','var'))
    disp('Aligning Pitch'); 
    yaw_comp = (e_jig(:,3).^4*pitch_pol_yaw.p1 + e_jig(:,3).^3*pitch_pol_yaw.p2 + e_jig(:,3).^2*pitch_pol_yaw.p3 + e_jig(:,3)*pitch_pol_yaw.p4 + pitch_pol_yaw.p5);
    roll_comp = (e_jig(:,3).^4*pitch_pol_roll.p1 + e_jig(:,3).^3*pitch_pol_roll.p2 + e_jig(:,3).^2*pitch_pol_roll.p3 + e_jig(:,3)*pitch_pol_roll.p4 + pitch_pol_roll.p5);
    pitch_comp = (e_jig(:,3).^4*pitch_pol_pitch.p1 + e_jig(:,3).^3*pitch_pol_pitch.p2 + e_jig(:,3).^2*pitch_pol_pitch.p3 + e_jig(:,3)*pitch_pol_pitch.p4 + pitch_pol_pitch.p5);

    e_bno(:,1) = e_bno(:,1)+yaw_comp;  
    e_bno(:,2) = e_bno(:,2)+roll_comp;  
    e_bno(:,3) = e_bno(:,3)+pitch_comp;

    e_madgwick(:,1) = e_madgwick(:,1)+yaw_comp; 
    e_madgwick(:,2) = e_madgwick(:,2)+roll_comp;  
    e_madgwick(:,3) = e_madgwick(:,3)+pitch_comp;

    e_mahony(:,1) = e_mahony(:,1)+yaw_comp;  
    e_mahony(:,2) = e_mahony(:,2)+roll_comp;  
    e_mahony(:,3) = e_mahony(:,3)+pitch_comp;

    e_CF(:,1) = e_CF(:,1)+yaw_comp;  
    e_CF(:,2) = e_CF(:,2)+roll_comp; 
    e_CF(:,3) = e_CF(:,3)+pitch_comp;

    e_gyroLib(:,1) = e_gyroLib(:,1)+yaw_comp;  
    e_gyroLib(:,2) = e_gyroLib(:,2)+roll_comp;  
    e_gyroLib(:,3) = e_gyroLib(:,3)+pitch_comp;
end

%% Erros
%calculate the quaternion difference btw the BNO and jig
% q_diff_bno = quatmultiply(quatconj(q_bno(1:length(q_jig),:)),q_jig);
% [e_diff_bno(:,1),e_diff_bno(:,2),e_diff_bno(:,3)] = quat2angle(q_diff_bno, Rot_Order);
%figure(11); plot(e_diff);

disp('Calculating errors');
e_diff_bno = angdiff(e_jig,e_bno); e_diff_bno(isnan(e_diff_bno)) = 0;
e_diff_madgwick = angdiff(e_jig,e_madgwick); e_diff_madgwick(isnan(e_diff_madgwick)) = 0;
e_diff_mahony = angdiff(e_jig,e_mahony); e_diff_mahony(isnan(e_diff_mahony)) = 0;
e_diff_CF = angdiff(e_jig,e_CF); e_diff_CF(isnan(e_diff_CF)) = 0;
e_diff_gyroLib = angdiff(e_jig,e_gyroLib); e_diff_gyroLib(isnan(e_diff_gyroLib)) = 0;

%% Fit rms and KTE (Kinematic Tracking Error)
kte_bno = sqrt(mean(abs(e_diff_bno*180/pi)).^2 + var(e_diff_bno*180/pi));
kte_madgwick = sqrt(mean(abs(e_diff_madgwick*180/pi)).^2 + var(e_diff_madgwick*180/pi));
kte_mahony = sqrt(mean(abs(e_diff_mahony*180/pi)).^2 + var(e_diff_mahony*180/pi));
kte_CF = sqrt(mean(abs(e_diff_CF*180/pi)).^2 + var(e_diff_CF*180/pi));
kte_gyroLib = sqrt(mean(abs(e_diff_gyroLib*180/pi)).^2 + var(e_diff_gyroLib*180/pi));

rms_bno = rms(e_diff_bno*180/pi);
rms_madgwick = rms(e_diff_madgwick*180/pi);
rms_mahony = rms(e_diff_mahony*180/pi);
rms_CF = rms(e_diff_CF*180/pi);
rms_gyroLib = rms(e_diff_gyroLib*180/pi);

fprintf('KTE: \r  BNO: %f; %f; %f \r  Madgwick: %f; %f; %f \r  Mahony: %f; %f; %f \r  CF: %f; %f; %f \r  Kalman: %f; %f; %f \r \r ',...
    kte_bno, kte_madgwick, kte_mahony, kte_CF, kte_gyroLib)
fprintf('RMS: \r  BNO: %f; %f; %f \r  Madgwick: %f; %f; %f \r  Mahony: %f; %f; %f \r  CF: %f; %f; %f \r  Kalman: %f; %f; %f \r \r ',...
    rms_bno, rms_madgwick, rms_mahony, rms_CF, rms_gyroLib)

fprintf('KTE MEAN: \r  BNO: %f; \r  Madgwick: %f; \r  Mahony: %f; \r  CF: %f;  \r  Kalman: %f; \r \r ',...
    mean(kte_bno), mean(kte_madgwick), mean(kte_mahony), mean(kte_CF), mean(kte_gyroLib))
fprintf('RMS MEAN: \r  BNO: %f; \r  Madgwick: %f; \r  Mahony: %f; \r  CF: %f; \r  Kalman: %f; \r \r ',...
    mean(rms_bno), mean(rms_madgwick), mean(rms_mahony), mean(rms_CF), mean(rms_gyroLib))
%% Plots
% figure; plot(t_match,e_diff_bno*180/pi); legend('Yaw','Roll','Pitch'); ylabel('Angle [°]');
% xlabel('time [s]'); axis([t_match(1) inf -inf inf]); title('Angle Error');
% 
% figure; 
% subplot(311);plot(t_imu,e_bno(:,1)*180/pi); hold all; plot(t_match,base_match); plot(t_match,e_diff_bno(:,1)*180/pi); axis([0 inf -200 200]); grid on;
% title('Time matched'); ylabel('yaw'); legend('BNO','Jig','error');
% subplot(312);plot(t_imu,e_bno(:,2)*180/pi);hold all; plot(t_imu,zeros(size(e_bno(:,2)))); plot(t_match,e_diff_bno(:,2)*180/pi); 
% axis([0 inf -inf inf]); grid on; ylabel('roll');
% subplot(313);plot(t_imu,e_bno(:,3)*180/pi);hold all; plot(t_match,-top_match); plot(t_match,e_diff_bno(:,3)*180/pi);axis([0 inf -200 200]); grid on;
% ylabel('pitch');

figure;
subplot(311);
plot(t_match,base_match,'Linewidth',2);hold all; 
plot(t_imu, e_bno(:,1)*180/pi,':','Linewidth',1.5) ;
plot(t_imu, e_madgwick(:,1)*180/pi);
plot(t_imu, e_mahony(:,1)*180/pi,'-.','Linewidth',1.5);
plot(t_imu, e_CF(:,1)*180/pi);
plot(t_imu, e_gyroLib(:,1)*180/pi,'--','Linewidth',1.5);
axis([0 inf -200 200]); grid on; title('Fusion comparison'); ylabel('Yaw [°]');
legend('Jig','BNO','Madgwick','Mahony','CF','Kalman','Orientation','Horizontal');
a = gca; a.XTickLabel = {};

subplot(312);
plot(t_imu,zeros(size(e_bno(:,2))),'Linewidth',2);hold all; 
plot(t_imu, e_bno(:,2)*180/pi,':','Linewidth',1.5) ;
plot(t_imu, e_madgwick(:,2)*180/pi);
plot(t_imu, e_mahony(:,2)*180/pi,'-.','Linewidth',1.5);
plot(t_imu, e_CF(:,2)*180/pi);
plot(t_imu, e_gyroLib(:,2)*180/pi,'--','Linewidth',1.5);
axis([0 inf -inf inf]); grid on; ylabel('Roll [°]');
a = gca; a.XTickLabel = {};

subplot(313);
plot(t_match,-top_match,'Linewidth',2);hold all; 
plot(t_imu, e_bno(:,3)*180/pi,':','Linewidth',1.5) ;
plot(t_imu, e_madgwick(:,3)*180/pi);
plot(t_imu, e_mahony(:,3)*180/pi,'-.','Linewidth',1.5);
plot(t_imu, e_CF(:,3)*180/pi);
plot(t_imu, e_gyroLib(:,3)*180/pi,'--','Linewidth',1.5);
axis([0 inf -200 200]); grid on; ylabel('Pitch [°]');

figure;
subplot(311);
plot(t_imu,[e_diff_bno(:,1) e_diff_madgwick(:,1) e_diff_mahony(:,1) e_diff_CF(:,1) e_diff_gyroLib(:,1)]*180/pi);
grid on; title('Fusion comparison error'); ylabel('yaw error [°]');
legend('BNO','Madgwick','Mahony','CF','Kalman','Orientation','Horizontal');
subplot(312);
plot(t_imu,[e_diff_bno(:,2) e_diff_madgwick(:,2) e_diff_mahony(:,2) e_diff_CF(:,2) e_diff_gyroLib(:,2)]*180/pi);
grid on; ylabel('roll error [°]');
subplot(313);
plot(t_imu,[e_diff_bno(:,3) e_diff_madgwick(:,3) e_diff_mahony(:,3) e_diff_CF(:,3) e_diff_gyroLib(:,3)]*180/pi);
grid on; ylabel('pitch error [°]');

%return 


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
plot(t_match,base_match,'Linewidth',2);hold all; 
plot(t_imu, e_bno(:,1)*180/pi,':','Linewidth',1.3) ;
plot(t_imu, e_madgwick(:,1)*180/pi);
plot(t_imu, e_mahony(:,1)*180/pi,'-.','Linewidth',1.5);
plot(t_imu, e_CF(:,1)*180/pi);
plot(t_imu, e_gyroLib(:,1)*180/pi,'--','Linewidth',1.5);
axis([0 65 -200 200]); grid on; title('Fusion comparison'); ylabel('Yaw [°]');
legend('Jig','BNO','Madgwick','Mahony','CF','Kalman','Orientation','Horizontal');
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(t_imu,zeros(size(e_bno(:,2))),'Linewidth',2);hold all; 
plot(t_imu, e_bno(:,2)*180/pi,':','Linewidth',1.3) ;
plot(t_imu, e_madgwick(:,2)*180/pi);
plot(t_imu, e_mahony(:,2)*180/pi,'-.','Linewidth',1.5);
plot(t_imu, e_CF(:,2)*180/pi);
plot(t_imu, e_gyroLib(:,2)*180/pi,'--','Linewidth',1.5);
axis([0 65 -inf inf]); grid on; ylabel('Roll [°]');
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(t_match,-top_match,'Linewidth',2);hold all; 
plot(t_imu, e_bno(:,3)*180/pi,':','Linewidth',1.3) ;
plot(t_imu, e_madgwick(:,3)*180/pi);
plot(t_imu, e_mahony(:,3)*180/pi,'-.','Linewidth',1.5);
plot(t_imu, e_CF(:,3)*180/pi);
plot(t_imu, e_gyroLib(:,3)*180/pi,'--','Linewidth',1.5);
axis([0 65 -200 200]); grid on; 
ylabel('Pitch [°]'); xlabel('Time [s]');
p(3).marginbottom = 10;


%error plot
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
plot(t_imu, e_diff_bno(:,1)*180/pi,':','Linewidth',1.3); hold all;
plot(t_imu, e_diff_madgwick(:,1)*180/pi);
plot(t_imu, e_diff_mahony(:,1)*180/pi,'-.','Linewidth',1.5);
plot(t_imu, e_diff_CF(:,1)*180/pi);
plot(t_imu, e_diff_gyroLib(:,1)*180/pi,'--','Linewidth',1.5);
axis([0 inf -inf inf]);
grid on; title('Fusion comparison'); ylabel('Yaw error [°]');
legend('BNO','Madgwick','Mahony','CF','Kalman','Orientation','Horizontal');
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(t_imu, e_diff_bno(:,2)*180/pi,':','Linewidth',1.3); hold all;
plot(t_imu, e_diff_madgwick(:,2)*180/pi);
plot(t_imu, e_diff_mahony(:,2)*180/pi,'-.','Linewidth',1.5);
plot(t_imu, e_diff_CF(:,2)*180/pi);
plot(t_imu, e_diff_gyroLib(:,2)*180/pi,'--','Linewidth',1.5);
axis([0 inf -inf inf]); grid on; ylabel('Roll error [°]');
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(t_imu, e_diff_bno(:,3)*180/pi,':','Linewidth',1.3); hold all;
plot(t_imu, e_diff_madgwick(:,3)*180/pi);
plot(t_imu, e_diff_mahony(:,3)*180/pi,'-.','Linewidth',1.5);
plot(t_imu, e_diff_CF(:,3)*180/pi);
plot(t_imu, e_diff_gyroLib(:,3)*180/pi,'--','Linewidth',1.5);
axis([0 inf -inf inf]); grid on; 
ylabel('Pitch error [°]'); xlabel('Time [s]');
p(3).marginbottom = 10;


positions = [1 1.15 1.3];
positions = [positions 0.75+positions];
figure('position',[300 300 800 450]);
boxplot([e_diff_gyroLib*180/pi e_diff_bno*180/pi],'positions', positions,...
    'OutlierSize', 1, 'Jitter',1);
set(gca,'xtick',[mean(positions(1:3)) mean(positions(4:6))])
set(gca,'xticklabel',{'Kalman','BNO'})
cc = lines(3);
color = repmat(cc,2,1);
h = findobj(gca,'Tag','Box');
for j=1:length(h)
   patch(get(h(j),'XData'),get(h(j),'YData'),color(j,:),'FaceAlpha',.5);
end
c = get(gca, 'Children');
hleg1 = legend(c(1:3), 'Yaw', 'Roll', 'Pitch','Orientation','Horizontal');
ylabel('Error [°]'); title('Kalman and BNO error distribution');
grid on;
%make the margins tight
% ax = gca;
% outerpos = ax.OuterPosition;
% ti = ax.TightInset; 
% left = outerpos(1) + ti(1);
% bottom = outerpos(2) + ti(2);
% ax_width = outerpos(3) - ti(1) - ti(3);
% ax_height = outerpos(4) - ti(2) - ti(4);
% ax.Position = [left bottom ax_width ax_height];


clear positions
positions = [1 1.15 1.3];
positions = [positions 0.75+positions 1.5+positions 2.25+positions 3+positions];
figure('position',[300 300 800 450]);
boxplot([e_diff_bno*180/pi e_diff_madgwick*180/pi e_diff_mahony*180/pi ...
    e_diff_CF*180/pi  e_diff_gyroLib*180/pi],'positions', positions,...
    'OutlierSize', 1, 'Jitter',1); %remove jitter to put everything in the same line
set(gca,'xtick',[mean(positions(1:3)) mean(positions(4:6)) mean(positions(7:9)) ...
                 mean(positions(10:12)) mean(positions(13:15))])
set(gca,'xticklabel',{'BNO','Madgwick','Mahony','CF','Kalman'})
cc = lines(3);
color = repmat(cc,5,1);
h = findobj(gca,'Tag','Box');
for j=1:length(h)
   patch(get(h(j),'XData'),get(h(j),'YData'),color(j,:),'FaceAlpha',.5);
end
c = get(gca, 'Children');
hleg1 = legend(c(1:3), 'Yaw', 'Roll', 'Pitch','Orientation','Horizontal');
ylabel('Error [°]'); title('DFMs error distribution');
%make the margins tight
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
grid on;



