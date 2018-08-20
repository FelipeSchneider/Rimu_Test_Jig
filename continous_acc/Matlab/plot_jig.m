run('compare_orientation_v2.m');
close all;
t_end = 70;
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
plot(t_match,base_match,'Linewidth',1.7); grid on; title('Jig Angles','fontsize',14);
ylabel('Yaw [°]','FontSize',12);
axis([0 t_end -190 190])
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(t_imu,zeros(size(e_bno(:,2))),'Linewidth',1.7); grid on;
ylabel('Roll [°]','FontSize',12);
axis([0 t_end -90 90])
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(t_match,-top_match,'Linewidth',1.7); grid on;
ylabel('Pitch [°]','FontSize',12);
axis([0 t_end -190 190])
xlabel('Time [s]','FontSize',12)
p(3).marginbottom = 10;


figure('pos',[10 10 960 540]);
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
plot(t_imu,[giro_imu_dps(1,1:end); giro_bno_dps(1,1:end)]'); grid on; title('Gyroscope','fontsize',14);
ylabel('X axis[dps]','FontSize',12); legend('LSM','BNO','Orientation','Horizontal');
axis([0 t_end -inf inf])
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(t_imu,[giro_imu_dps(2,1:end); giro_bno_dps(2,1:end)]'); grid on;
ylabel('Y axis[dps]','FontSize',12);
axis([0 t_end -inf inf])
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(t_imu,[giro_imu_dps(3,1:end); giro_bno_dps(3,1:end)]'); grid on;
ylabel('Z axis[dps]','FontSize',12);
axis([0 t_end -inf inf])
xlabel('Time [s]','FontSize',12)
p(3).marginbottom = 10;

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
plot(t_imu,[acc_imu_g(1,1:end); acc_bno_g(1,1:end)]'); grid on; title('Accelerometer','fontsize',14);
ylabel('X axis[g]','FontSize',12); legend('LSM','BNO','Orientation','Horizontal');
axis([0 t_end -inf inf])
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(t_imu,[acc_imu_g(2,1:end); acc_bno_g(2,1:end)]'); grid on;
ylabel('Y axis[g]','FontSize',12);
axis([0 t_end -inf inf])
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(t_imu,[acc_imu_g(3,1:end); acc_bno_g(3,1:end)]'); grid on;
ylabel('Z axis[g]','FontSize',12);
axis([0 t_end -inf inf])
xlabel('Time [s]','FontSize',12)
p(3).marginbottom = 10;
