addpath('C:\Users\felip\Dropbox\Mestrado\Dissertação\Coletas Jiga\Teste_giro_filt')  %just to be able to load previously jig measurements
addpath(genpath('..\fusion'));
addpath('..\jig');
addpath('..\');
% addpath('..\aquire_rimu');
load fusion4
clearvars -except acc_bno_g acc_imu_g com_data description fs giro_bno_dps giro_imu_dps ...
    jig_const mag_bno_gaus mag_imu_gaus Q real_base_angle real_top_angle t...
    t_angles t_imu t_rec
close all;
%% Genetic Algorithms parameters
plot_figure = 1;
n_particles = 15;
nIterations = 100;
alpha = 0.25;
r_mutation = 0.05;
% Xmin = [10e-3   10e-3   10e-4   10e-6   10e-6   10e-6]; %limits of search
% Xmax = [1       1       10e-1   10e-1   10e-2   10e-4];

% Xmin = [5e-2   5e-2   5e-3   5e-5   5e-5   5e-8]; %limits of search
% Xmax = [5e-1   5e-1   5e-2   5e-4   5e-4   5e-7];
Xmin = [1e-2   1e-2   1e-4   1e-6   5e-5   5e-8]; %limits of search
Xmax = [2e-1   2e-1   1e-2   1e-4   5e-4   5e-7];
%X(1) -> Three values of superior diag. of R -> R(1,1);R(2,2);R(3,3).
%X(2) -> Three values of inferior diag. of R -> R(4,4);R(5,5);R(6,6).
%X(3) -> Three values of superior diag. of P -> P(1,1);P(2,2);P(3,3).
%X(4) -> Three values of inferior diag. of P -> P(4,4);P(5,5);P(6,6).
%X(5) -> gyro error noise
%X(6) -> gyro bias noise
%original values [1e-1 1e-1 1e-3 1e-5 1e-4 1e-7]
%R is the measurement noise covariance matrix
%P is the error covariance matrix
%% Data that must be pre entered
%Kalman variables
dWb = (giro_bno_dps); 
Fb  = acc_bno_g; 
Mb  = mag_bno_gaus;
%correcting the referencial to the original code referencial
% Fb(3,:) = -Fb(3,:); Fb(2,:) = -Fb(2,:);
% dWb(3,:) = -dWb(3,:); dWb(2,:) = -dWb(2,:);
% Mb(3,:) = -Mb(3,:); Mb(2,:) = -Mb(2,:);
fn  = mean(Fb(:,100:400),2);
mn  = mean(Mb(:,100:400),2);
bw = mean(dWb(:,100:400),2);
fs = 100;                                   %sampling rate

%bw = [2.5052;   -5.7301;   -4.8365]*pi/180; %gyro bias
% fn = mean(acc_imu_g(:,100:400),2);          %gravity in the sensor frame
% mn = mean(mag_bno_gaus(:,100:400),2);       %magnetic field in the sensor frame


%magnetometer calibration
[mag_imu_gaus_cal, Ca_imu, Cb_imu] = magCalibration(mag_imu_gaus');
mag_imu_gaus_cal = mag_imu_gaus_cal';

%% Compensating the jig time and 
t_rec(1) = [];                  %the first position is a zero, used only for prealocation
t_expect = com_data(1,:);       %extracting the expect time for each command
t_expect(end+1) = t(end);       %filling with the end of all comands here
t = t-t_rec(1);

comp_factor = 123.5/123.0383;    %took from graph FUSION4
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

if plot_figure == 1
    figure(1)
    subplot(4,2,[1 3 5]); %subplot of the numerator
    left_graph = scatter3(X(1,:),X(2,:),X(3,:),'*b','linewidth',1);
    axis square;grid on;
    axis([Xmin(1) Xmax(1)*1.1 Xmin(2) Xmax(2)*1.1 Xmin(3) Xmax(3)*1.1]); %.1 prevents the 0,0 axis
    
    subplot(4,2,[2 4 6]); %subplot of the Denominator
    right_graph = scatter3(X(4,:),X(5,:),X(6,:),'*b','linewidth',1);
    axis square;grid on;
    axis([Xmin(4) Xmax(4)*1.1 Xmin(5) Xmax(5)*1.1 Xmin(6) Xmax(6)*1.1]);
end
%% First GA iteration
%function value calculation for each particle
%[q_out] = Kalman_response(X, Fb, dWb, Mb, bw, fn, mn, fs);
[q_out] = Kalman_response(X,acc_bno_g, giro_bno_dps, mag_bno_gaus,bw, fn, mn, fs);
[fit] = Kalman_fit(q_out,q_jig);
