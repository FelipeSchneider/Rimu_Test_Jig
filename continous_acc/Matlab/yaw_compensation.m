

run compare_orientation_v2.m %make sure that here we load the correct file
%close all

figure;
subplot(311);
plot(t_imu,[e_diff_bno(:,1) e_diff_madgwick(:,1) e_diff_mahony(:,1)  e_diff_gyroLib(:,1) e_diff_CF(:,1)]*180/pi);
hold on; plot(t_rec,zeros(1,length(t_rec)),'*');
grid on; title('Fusion comparison error'); ylabel('yaw error [°]');
legend('BNO','Madgwick','Mahony','Kalman','CF','Orientation','Horizontal');
subplot(312);
plot(t_imu,[e_diff_bno(:,2) e_diff_madgwick(:,2) e_diff_mahony(:,2) e_diff_gyroLib(:,2) e_diff_CF(:,2)]*180/pi);
grid on; ylabel('roll error [°]');
subplot(313);
plot(t_imu,[e_diff_bno(:,3) e_diff_madgwick(:,3) e_diff_mahony(:,3)  e_diff_gyroLib(:,3) e_diff_CF(:,3)]*180/pi);
grid on; ylabel('pitch error [°]');

%ang = -180:10:180; ang = ang*pi/180;
for i=1:length(t_rec)/2
    [~, index] = min(abs(t_rec(i*2)-t_imu));
    ang(i) = mean(e_jig(index-200:index-100,1)); %yaw correction
    
    e_mean_diff_bno(i,:) = mean(e_diff_bno(index-200:index-100,:));
    e_mean_diff_kalman(i,:) = mean(e_diff_gyroLib(index-200:index-100,:));
    e_mean_diff_mahony(i,:) = mean(e_diff_mahony(index-200:index-100,:));
    e_mean_diff_madgwick(i,:) = mean(e_diff_madgwick(index-200:index-100,:));
    e_mean_diff_CF_imu(i,:) = mean(e_diff_CF(index-200:index-100,:));
    
    error_mean(i,:) = mean([e_mean_diff_bno(i,:);e_mean_diff_kalman(i,:);...
        e_mean_diff_mahony(i,:);e_mean_diff_madgwick(i,:);e_mean_diff_CF_imu(i,:)]);
end






[pol_yaw,S_yaw] = fit(ang',error_mean(:,1),'poly4');
[pol_roll,S_roll] = fit(ang',error_mean(:,2),'poly4');
[pol_pitch,S_pitch] = fit(ang',error_mean(:,3),'poly4');
S_yaw
S_roll
S_pitch
% figure; plot(pol_yaw, ang', error_mean(:,1),'residuals'); hold on; plot(pol_yaw, 'predobs');




cor = lines(7);
% figure;
% subplot(311);
% plot(ang,e_mean_diff_bno(:,1),'.','MarkerSize',10,'color',cor(2,:)); hold all; 
% plot(ang,e_mean_diff_kalman(:,1),'.','MarkerSize',10,'color',cor(3,:)); 
% plot(ang,e_mean_diff_mahony(:,1),'.','MarkerSize',10,'color',cor(4,:)); 
% plot(ang,e_mean_diff_madgwick(:,1),'.','MarkerSize',10,'color',cor(5,:)); 
% plot(ang,e_mean_diff_CF_imu(:,1),'.','MarkerSize',10,'color',cor(6,:));
% plot(ang, error_mean(:,1),'.','MarkerSize',12,'color',cor(7,:)); 
% plot(pol_yaw, 'predobs'); axis([-pi pi -.15 .15]);
% legend('BNO','Kalman','Mahony','Madgwick','CF','Mean','Orientation','Horizontal');
% grid on;
% %a = gca; a.XTickLabel = {};
% 
% subplot(312);
% plot(ang,e_mean_diff_bno(:,2),'.','MarkerSize',10,'color',cor(2,:)); hold all; 
% plot(ang,e_mean_diff_kalman(:,2),'.','MarkerSize',10,'color',cor(3,:)); 
% plot(ang,e_mean_diff_mahony(:,2),'.','MarkerSize',10,'color',cor(4,:)); 
% plot(ang,e_mean_diff_madgwick(:,2),'.','MarkerSize',10,'color',cor(5,:)); 
% plot(ang,e_mean_diff_CF_imu(:,2),'.','MarkerSize',10,'color',cor(6,:));
% plot(ang, error_mean(:,2),'.','MarkerSize',12,'color',cor(7,:)); 
% plot(pol_roll, 'predobs'); axis([-pi pi -.05 0.075]); legend('off')
% grid on;
% %a = gca; a.XTickLabel = {};
% 
% subplot(313);
% plot(ang,e_mean_diff_bno(:,3),'.','MarkerSize',10,'color',cor(2,:)); hold all; 
% plot(ang,e_mean_diff_kalman(:,3),'.','MarkerSize',10,'color',cor(3,:)); 
% plot(ang,e_mean_diff_mahony(:,3),'.','MarkerSize',10,'color',cor(4,:)); 
% plot(ang,e_mean_diff_madgwick(:,3),'.','MarkerSize',10,'color',cor(5,:)); 
% plot(ang,e_mean_diff_CF_imu(:,3),'.','MarkerSize',10,'color',cor(6,:));
% plot(ang, error_mean(:,3),'.','MarkerSize',12,'color',cor(7,:)); 
% plot(pol_pitch, 'predobs'); axis([-pi pi -0.1 0.02]); legend('off')
% grid on; 








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
plot(ang,e_mean_diff_bno(:,1),'.','MarkerSize',10,'color',cor(2,:)); hold all; 
plot(ang,e_mean_diff_kalman(:,1),'.','MarkerSize',10,'color',cor(3,:)); 
plot(ang,e_mean_diff_mahony(:,1),'.','MarkerSize',10,'color',cor(4,:)); 
plot(ang,e_mean_diff_madgwick(:,1),'.','MarkerSize',10,'color',cor(5,:)); 
plot(ang,e_mean_diff_CF_imu(:,1),'.','MarkerSize',10,'color',cor(6,:));
plot(ang, error_mean(:,1),'.','MarkerSize',12,'color',cor(7,:)); 
plot(pol_yaw, 'predobs'); ylabel('Rimu Yaw [rad]');
axis([-pi pi -.15 .15]);
legend('BNO','Kalman','Mahony','Madgwick','CF','Mean','Orientation','Horizontal');
grid on;
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(ang,e_mean_diff_bno(:,2),'.','MarkerSize',10,'color',cor(2,:)); hold all; 
plot(ang,e_mean_diff_kalman(:,2),'.','MarkerSize',10,'color',cor(3,:)); 
plot(ang,e_mean_diff_mahony(:,2),'.','MarkerSize',10,'color',cor(4,:)); 
plot(ang,e_mean_diff_madgwick(:,2),'.','MarkerSize',10,'color',cor(5,:)); 
plot(ang,e_mean_diff_CF_imu(:,2),'.','MarkerSize',10,'color',cor(6,:));
plot(ang, error_mean(:,2),'.','MarkerSize',12,'color',cor(7,:)); 
plot(pol_roll, 'predobs');  ylabel('Rimu Roll [rad]');
axis([-pi pi -.05 0.075]); legend('off')
grid on;
a = gca; a.XTickLabel = {};

p(3,1,1).select();
plot(ang,e_mean_diff_bno(:,3),'.','MarkerSize',10,'color',cor(2,:)); hold all; 
plot(ang,e_mean_diff_kalman(:,3),'.','MarkerSize',10,'color',cor(3,:)); 
plot(ang,e_mean_diff_mahony(:,3),'.','MarkerSize',10,'color',cor(4,:)); 
plot(ang,e_mean_diff_madgwick(:,3),'.','MarkerSize',10,'color',cor(5,:)); 
plot(ang,e_mean_diff_CF_imu(:,3),'.','MarkerSize',10,'color',cor(6,:));
plot(ang, error_mean(:,3),'.','MarkerSize',12,'color',cor(7,:)); 
plot(pol_pitch, 'predobs'); xlabel('TJ Yaw [rad]'); ylabel('Rimu Pitch [rad]');
axis([-pi pi -0.1 0.02]); legend('off')
grid on; 
p(3).marginbottom = 10;