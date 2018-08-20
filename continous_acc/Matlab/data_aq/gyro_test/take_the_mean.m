clear all; close all; clc;

load('giro_test_filt2.mat')
giro_imu_filt = giro_imu_dps - [2.1627;-5.7153;-4.9309];
giro_bno_filt = giro_bno_dps;
acc_imu_filt = acc_imu_g;
acc_bno_filt = acc_bno_g;
load('giro_test_non_filt2.mat')
giro_imu_non_filt = giro_imu_dps - [2.1627;-5.7153;-4.9309];
giro_bno_non_filt = giro_bno_dps;
acc_imu_non_filt = acc_imu_g;
acc_bno_non_filt = acc_bno_g;

    



inicio = 200;
fim = 1500;
m_60 = mean([giro_imu_filt(2,inicio:fim);giro_bno_dps(2,inicio:fim)]');

inicio = 180;
fim = 3200;
m_120 = mean([giro_imu_filt(2,inicio:fim);giro_bno_dps(2,inicio:fim)]');

inicio = 3800;
fim = 5200;
m_180 = mean([giro_imu_filt(2,inicio:fim);giro_bno_dps(2,inicio:fim)]');

inicio = 5700;
fim = 7000;
m_240 = mean([giro_imu_filt(2,inicio:fim);giro_bno_dps(2,inicio:fim)]');

inicio = 7500;
fim = 8900;
m_360 = mean([giro_imu_filt(2,inicio:fim);giro_bno_dps(2,inicio:fim)]');

inicio = 9500;
fim = 10200;
m_540 = mean([giro_imu_filt(2,inicio:fim);giro_bno_dps(2,inicio:fim)]');

inicio = 10900;
fim = 11400;
m_720 = mean([giro_imu_filt(2,inicio:fim);giro_bno_dps(2,inicio:fim)]');

%%
figure(30);
subplot(311);plot(t_imu,[giro_imu_non_filt(1,1:n_sample); giro_imu_filt(1,1:n_sample);giro_bno_filt(1,1:n_sample)]'); grid on;
title('Gyroscope measurements');ylabel('X axis [dps]');legend('LSM non-filt','LSM filtered','BNO');
a = gca; a.XTickLabel = {};
subplot(312);plot(t_imu,[giro_imu_non_filt(2,1:n_sample); giro_imu_filt(2,1:n_sample);giro_bno_filt(2,1:n_sample)]'); ylabel('Y axis [dps]'); grid on;
a = gca; a.XTickLabel = {};
subplot(313);plot(t_imu,[giro_imu_non_filt(3,1:n_sample); giro_imu_filt(3,1:n_sample);giro_bno_filt(3,1:n_sample)]'); ylabel('Z axis [dps]'); grid on;
xlabel('Time [s]')

%%
figure;
p = panel();
p.pack('v',3);
p(1).pack({1}, {70 30});
p(2).pack({1}, {70 30});
p(3).pack({1}, {70 30});
p.de.margin = 12;
p.margin = [15 10 5 5];
p.select('all');
p.fontsize = 10;
p.identify();


p(1,1,1).select();
plot(t_imu,[giro_imu_non_filt(1,1:n_sample); giro_imu_filt(1,1:n_sample);giro_bno_filt(1,1:n_sample)]'); grid on;
ylabel('X axis [dps]');%legend('LSM non-filt','LSM filtered','BNO');
axis([0 120 -25 25]); grid on;
a = gca; a.XTickLabel = {};
p(1,1,2).select();
plot(t_imu,[giro_imu_non_filt(1,1:n_sample); giro_imu_filt(1,1:n_sample);giro_bno_filt(1,1:n_sample)]'); grid on;
%axis([65 70 -25 25]); grid on;
a = gca; a.YTickLabel = {};a.XTickLabel = {};
p(1,1,1).margin = [3 3 3 3];
p(1,1,2).margin = [3 3 3 3];

p(2,1,1).select();
plot(t_imu,[giro_imu_non_filt(2,1:n_sample); giro_imu_filt(2,1:n_sample);giro_bno_filt(2,1:n_sample)]'); grid on;
ylabel('Y axis [dps]');%legend('LSM non-filt','LSM filtered','BNO');
%axis([0 101 -750 10]); grid on;
a = gca; a.XTickLabel = {};
p(2,1,2).select();
plot(t_imu,[giro_imu_non_filt(2,1:n_sample); giro_imu_filt(2,1:n_sample);giro_bno_filt(2,1:n_sample)]'); grid on;
%axis([65 70 -750 10]); grid on;
a = gca; a.YTickLabel = {};a.XTickLabel = {};
p(2,1,1).margin = [3 3 3 3];
p(2,1,2).margin = [3 3 3 3];

p(3,1,1).select();
plot(t_imu,[giro_imu_non_filt(3,1:n_sample); giro_imu_filt(3,1:n_sample);giro_bno_filt(3,1:n_sample)]'); grid on;
ylabel('Y axis [dps]');%legend('LSM non-filt','LSM filtered','BNO');
%axis([0 101 -15 25]); grid on;
p(3,1,2).select();
plot(t_imu,[giro_imu_non_filt(3,1:n_sample); giro_imu_filt(3,1:n_sample);giro_bno_filt(3,1:n_sample)]'); grid on;
%axis([65 70 -15 25]); grid on;
a = gca; a.YTickLabel = {};
p(3,1,1).margin = [3 3 3 3];
p(3,1,2).margin = [3 3 3 3];


p(1).marginbottom = 3;
p(2).marginbottom = 3;
p(1).margintop = 3;
p(2).margintop = 3;
p(3).margintop = 3;


%%
figure('position',[400 400 665 440]);
p = panel();
p.pack('v',3);
p(1).pack({1}, {100});
p(2).pack({1}, {100});
p(3).pack({1}, {100});
p.de.margin = 12;
p.margin = [15 15 5 5];
p.select('all');
p.fontsize = 10;
p.identify();


p(1,1,1).select();
plot(t_imu,[giro_imu_non_filt(1,1:n_sample); giro_imu_filt(1,1:n_sample);giro_bno_filt(1,1:n_sample)]'); grid on;
ylabel('X axis [dps]','FontSize',12');%legend('LSM non-filt','LSM filtered','BNO');
axis([0 120 -25 25]); grid on;
a = gca; a.XTickLabel = {};
legend('LSM non-filtered','LSM filtered','BNO','Location','north','Orientation','horizontal')
%legend('boxoff');
p(2,1,1).select();
plot(t_imu,[giro_imu_non_filt(2,1:n_sample); giro_imu_filt(2,1:n_sample);giro_bno_filt(2,1:n_sample)]'); grid on;
ylabel('Y axis [dps]','FontSize',12);%legend('LSM non-filt','LSM filtered','BNO');
axis([0 120 -10 750]); grid on;
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(t_imu,[giro_imu_non_filt(3,1:n_sample); giro_imu_filt(3,1:n_sample);giro_bno_filt(3,1:n_sample)]'); grid on;
ylabel('Y axis [dps]','FontSize',12);%legend('LSM non-filt','LSM filtered','BNO');
xlabel('Time [s]','FontSize',12);
axis([0 120 -15 10]); grid on;

p(1).marginbottom = 3;
p(2).marginbottom = 3;
p(1).margintop = 3;
p(2).margintop = 3;
p(3).margintop = 3;

%Jig angular acceleration fit module
[P,S] = fit(t_imu(87.71*100:88.55*100)',giro_bno_filt(2,87.71*100:88.55*100)','poly1')

%%
figure('position',[300 300 665 440]);
p = panel();
p.pack('v',3);
p(1).pack({1}, {100});
p(2).pack({1}, {100});
p(3).pack({1}, {100});
p.de.margin = 12;
p.margin = [15 15 5 5];
p.select('all');
p.fontsize = 10;
p.identify();


p(1,1,1).select();
plot(t_imu,[acc_imu_non_filt(1,1:n_sample); acc_imu_filt(1,1:n_sample);acc_bno_filt(1,1:n_sample)]'); grid on;
ylabel('X axis [g]','FontSize',12');%legend('LSM non-filt','LSM filtered','BNO');
axis([0 120 -2 2]); grid on;
a = gca; a.XTickLabel = {};
legend('LSM non-filtered','LSM filtered','BNO','Location','north','Orientation','horizontal')


p(2,1,1).select();
plot(t_imu,[acc_imu_non_filt(2,1:n_sample); acc_imu_filt(2,1:n_sample);acc_bno_filt(2,1:n_sample)]'); grid on;
ylabel('Y axis [g]','FontSize',12);%legend('LSM non-filt','LSM filtered','BNO');
axis([0 120 -.4 .4]); grid on;
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(t_imu,[acc_imu_non_filt(3,1:n_sample); acc_imu_filt(3,1:n_sample);acc_bno_filt(3,1:n_sample)]'); grid on;
ylabel('Z axis [g]','FontSize',12);%legend('LSM non-filt','LSM filtered','BNO');
xlabel('Time [s]','FontSize',12);
axis([0 120 -2 2]); grid on;

p(1).marginbottom = 3;
p(2).marginbottom = 3;
p(1).margintop = 3;
p(2).margintop = 3;
p(3).margintop = 3;

%%
Q_acc = Q(:,8600:8900);
[e_acc(:,1),e_acc(:,2),e_acc(:,3)] = quat2angle(Q_acc','ZXY');


%Jig angular acceleration fit module
[P,S] = fit([0;m_60(1);m_120(1);m_180(1);m_240(1);m_360(1);m_540(1);m_720(1)],...
    [0;60;120;180;240;360;540;720],'poly1')

w_fit = giro_bno_filt(2,10641:10839);
t_fit = 0:0.01:length(giro_bno_filt(2,10641:10839))*0.01-0.01;
[P_acc,S_acc] = fit(t_fit',w_fit','poly1')