

run compare_orientation_v2.m %make sure that here we load the correct file
%close all

figure;
subplot(311);
plot(t_imu,[e_diff_bno(:,1) e_diff_madgwick_imu(:,1) e_diff_mahony_imu(:,1)  e_diff_gyroLib(:,1) e_diff_CF_imu(:,1)]*180/pi);
hold on; plot(t_rec,zeros(1,length(t_rec)),'*');
grid on; title('Fusion comparison error'); ylabel('yaw error [°]');
legend('BNO','Madgwick','Mahony','Kalman','CF','Orientation','Horizontal');
subplot(312);
plot(t_imu,[e_diff_bno(:,2) e_diff_madgwick_imu(:,2) e_diff_mahony_imu(:,2) e_diff_gyroLib(:,2) e_diff_CF_imu(:,2)]*180/pi);
grid on; ylabel('roll error [°]');
subplot(313);
plot(t_imu,[e_diff_bno(:,3) e_diff_madgwick_imu(:,3) e_diff_mahony_imu(:,3)  e_diff_gyroLib(:,3) e_diff_CF_imu(:,3)]*180/pi);
grid on; ylabel('pitch error [°]');

%ang = -180:10:180; ang = ang*pi/180;
for i=1:length(t_rec)/2
    [~, index] = min(abs(t_rec(i*2)-t_imu));
    ang(i) = mean(e_jig(index-200:index-100,1)); %yaw correction
    
    e_mean_diff_bno(i,:) = mean(e_diff_bno(index-200:index-100,:));
    e_mean_diff_kalman(i,:) = mean(e_diff_gyroLib(index-200:index-100,:));
    e_mean_diff_mahony(i,:) = mean(e_diff_mahony_imu(index-200:index-100,:));
    e_mean_diff_madgwick(i,:) = mean(e_diff_madgwick_imu(index-200:index-100,:));
    e_mean_diff_CF_imu(i,:) = mean(e_diff_CF_imu(index-200:index-100,:));
    
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

figure; subplot(311);
plot(ang,[e_mean_diff_bno(:,1) e_mean_diff_kalman(:,1) e_mean_diff_mahony(:,1) e_mean_diff_madgwick(:,1) e_mean_diff_CF_imu(:,1)],'.','MarkerSize',15)
hold on; plot(ang, error_mean(:,1),'.','MarkerSize',20); hold on; plot(pol_yaw, 'predobs');
legend('BNO','Kalman','Mahony','Madgwick','CF','Mean','Orientation','Horizontal');
grid on;
subplot(312);
plot(ang,[e_mean_diff_bno(:,2) e_mean_diff_kalman(:,2) e_mean_diff_mahony(:,2) e_mean_diff_madgwick(:,2) e_mean_diff_CF_imu(:,2)],'.','MarkerSize',15)
hold on; plot(ang, error_mean(:,2),'.','MarkerSize',20); hold on; plot(pol_roll, 'predobs');
grid on;
subplot(313);
plot(ang,[e_mean_diff_bno(:,3) e_mean_diff_kalman(:,3) e_mean_diff_mahony(:,3) e_mean_diff_madgwick(:,3) e_mean_diff_CF_imu(:,3)],'.','MarkerSize',15)
hold on; plot(ang, error_mean(:,3),'.','MarkerSize',20); hold on; plot(pol_pitch, 'predobs');
grid on;