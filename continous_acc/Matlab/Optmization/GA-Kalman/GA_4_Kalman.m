%addpath('C:\Users\felip\Dropbox\Mestrado\Dissertação\Coletas Jiga\Teste_giro_filt')  %just to be able to load previously jig measurements
% addpath(genpath('C:\Users\felip\Documents\Arquivos dissertação\Testes dissertação\data colection'))  %just to be able to load previously jig measurements
addpath(genpath('C:\Users\felip\Documents\Arquivos dissertação\Testes dissertação\_new data collection'))
addpath(genpath('..\..\fusion'));
addpath('..\..\jig');
addpath('..\..\');
% addpath('..\aquire_rimu');
%load random2
%load mid_range_data6.mat   %o 6 ficou com erro baixo
load fast_range_data4.mat   %o 4 é o melhor
%load slow_range_data.mat

clearvars -except acc_bno_g acc_imu_g com_data description fs giro_bno_dps giro_imu_dps ...
    jig_const mag_bno_gaus mag_imu_gaus Q real_base_angle real_top_angle t...
    t_angles t_imu t_rec

load LIS_cal_mat
load Yaw_alignment
load Pitch_alignment

clc; close all;

A = {pol_yaw, pol_roll, pol_pitch, pitch_pol_yaw, pitch_pol_roll, pitch_pol_pitch};
%% calibrate LSM gyro data
giro_imu_dps = giro_imu_dps - mean(giro_imu_dps(:,200:350),2);
giro_imu_dps = giro_imu_dps*1.1692104;

%% calibrate LIS data
mag_imu_gaus_cal = mag_imu_gaus' * Ca_imu' + repmat(Cb_imu', length(mag_imu_gaus), 1);
mag_imu_gaus_cal = mag_imu_gaus_cal';
%% Genetic Algorithms parameters
plot_figures = 1;
n_particles = 40;
nIterations = 20;
alpha = 0.25;
r_mutation = 0.1;
% Xmin = [10e-3   10e-3   10e-4   10e-6   10e-6   10e-6]; %limits of search
% Xmax = [1       1       10e-1   10e-1   10e-2   10e-4];

Xmin = [5e-3   5e-3   5e-5   5e-5   5e-5   5e-8]; %limits of search
Xmax = [5e-1   5e-1   5e-2   5e-2   5e-3   5e-6]*2;
%X(1) -> Three values of superior diag. of R -> R(1,1);R(2,2);R(3,3). measurement noise covariance matrix
%X(2) -> Three values of inferior diag. of R -> R(4,4);R(5,5);R(6,6). measurement noise covariance matrix
%X(3) -> Three values of superior diag. of P -> P(1,1);P(2,2);P(3,3). error covariance matrix
%X(4) -> Three values of inferior diag. of P -> P(4,4);P(5,5);P(6,6). error covariance matrix
%X(5) -> gyro error noise
%X(6) -> gyro bias noise
%original values [1e-1 1e-1 1e-2 1e-4 1e-4 1e-7]
%% Data that must be pre entered
Rot_Order = 'ZXY';
acc_data = (acc_imu_g + acc_bno_g)/2;
giro_data = (giro_imu_dps + giro_bno_dps)/2;
mag_data = (mag_bno_gaus+mag_imu_gaus_cal)/2;
avg_time_quat = 200:350;
avg_time_sens = 200:350;
% acc_data = acc_imu_g;
% giro_data = giro_imu_dps;
% mag_data = mag_bno_gaus;

%Kalman constants
fs = 100;                                   %sampling rate
bw = mean(giro_data(:,avg_time_sens),2);          %gyro bias
fn = mean(acc_data(:,avg_time_sens),2);           %gravity in the sensor frame
mn = mean(mag_data(:,avg_time_sens),2);           %magnetic field in the sensor frame

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


%% GA CODE
n=0;
n_mutations = round(r_mutation*n_particles);    %number of mutations
n_mutations = max(1,n_mutations);               %minimum number of mutations
n_parameters = length(Xmin);                    %number of parameters for each particle

% initialization using uniform probability distribution for each particle
X = zeros(n_parameters,n_particles); 
Pbest = zeros(n_parameters,n_particles);
for k = 1:n_particles
    for j=1:n_parameters
        X(j,k) = Xmin(j) + (Xmax(j)-Xmin(j))*rand(1);
    end
end

%[ left_particles, right_particles ] = plotParticles(X, Xmin, Xmax, plot_figures);

%% First GA iteration
%function value calculation for each particle
fprintf('GA particle initialization \n')
[q_out, X] = Kalman_response(X, acc_data, giro_data, mag_data, bw, fn, mn, fs, Xmin, Xmax);
[fit] = Kalman_fit(q_out, q_jig, A);

%find the best particle
[fGbest,id_best] = min(fit);
Gbest = X(:,id_best);
[ HP_particles ] = plotParticles(X, Gbest, Xmin, Xmax, plot_figures);
[ HP_estimation ] = plotEstimation(t_imu, q_out, q_out(:,:,id_best), q_jig, plot_figures, A);
%% Second to last GA iteration
for n=1:nIterations
    fprintf('GA iteration number %d \r',n)
    [X,fit] = updateX(X,alpha, q_jig, Xmin, Xmax, acc_data, giro_data, mag_data, bw, fn, mn, fs, A);
    [X,Gbest,fGbest,id_best] = FindBest(X, fit, fGbest, Gbest);

    fprintf('RMS best fit: %f \r',fGbest);
    [X] = mutation(X, n_mutations, id_best, Xmin, Xmax);
    [ ~ ] = plotParticles(X, Gbest, Xmin, Xmax, plot_figures, n, HP_particles);
    [ ~ ] = plotEstimation(t_imu, q_out, q_out(:,:,id_best), q_jig, plot_figures, A, n, HP_estimation);
end