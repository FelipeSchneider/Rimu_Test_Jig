addpath('C:\Users\felip\Dropbox\Mestrado\Dissertação\Coletas Jiga\Teste_giro_filt')  %just to be able to load previously jig measurements
load fusion6
%% Comparing the results
%compensating the jig time
t_rec(1) = [];                  %the first position is a zero, used only for prealocation
t_expect = com_data(1,:);       %extracting the expect time for each command
t_expect(end+1) = t(end);       %filling with the end of all comands here
t = t-t_rec(1);
%comp_factor = 123.5/123.0383;   %took from graph FUSION4
%comp_factor = 56.7/56.8678;    %took from graph FUSION5
comp_factor = 254.13/253.07818;                %FUSION 6
t_jig_comp = t*comp_factor;

%just to plot the marks on the graph
b_angle_expect = com_data(2,:); b_angle_expect(end+1) = real_base_angle(end);
t_angle_expect = com_data(3,:); t_angle_expect(end+1) = real_top_angle(end);

%align the board with the jig
Q_norm = quatnormalize(Q');
Q_zero = avg_quaternion_markley(Q_norm(150:450,:));
Q_norm = quatmultiply(Q_norm, quatconj(Q_zero'));
Q_norm = quatnormalize(Q_norm);

%wrap the aligned BNO angles around -180 and 180 degrees
[e_bno(:,1),e_bno(:,2),e_bno(:,3)] = quat2angle(Q_norm,'ZXY');
b_angle = wrapTo180(real_base_angle); b_angle_expect = wrapTo180(b_angle_expect);
t_angle = wrapTo180(real_top_angle);  t_angle_expect = wrapTo180(t_angle_expect);
%e_bno(end,:) = []; b_angle(end) = [];  t_angle(end) = [];

%match the imu and jig times
[t_match, base_match, top_match] = matchJigIMU(t_jig_comp, t_imu, b_angle, t_angle);
Q_jig = zeros(length(t_match),4);
for i=1:length(top_match)
    Q_jig(i,:) = angle2quat(base_match(i)*pi/180, 0, -top_match(i)*pi/180,'ZXY');
end
% [e_jig(:,1),e_jig(:,2),e_jig(:,3)] = quat2angle(Q_jig,'ZXY');
% figure(10); plot(e_jig);

%calculate the quaternion difference btw the BNO and jig
Q_diff = quatmultiply(quatconj(Q_norm),Q_jig);
[e_diff(:,1),e_diff(:,2),e_diff(:,3)] = quat2angle(Q_diff,'ZXY');
%figure(11); plot(e_diff);

%% Plots again
figure; 
subplot(311);plot(t_imu,e_bno(:,1)*180/pi); hold all; plot(t,b_angle); plot(t_expect,b_angle_expect,'r*');
title('Non-compensated'); legend('BNO','Jig','expectation');
subplot(312);plot(t_imu,e_bno(:,2)*180/pi);
subplot(313);plot(t_imu,e_bno(:,3)*180/pi);hold all; plot(t,-t_angle); plot(t_expect,-t_angle_expect,'r*');

figure; 
subplot(311);plot(t_imu,e_bno(:,1)*180/pi); hold all; plot(t_jig_comp,b_angle); plot(t_rec,b_angle_expect,'r*'); axis([0 inf -200 200]); grid on;
title('Compensated'); ylabel('yaw'); legend('BNO','Jig','expectation');
subplot(312);plot(t_imu,e_bno(:,2)*180/pi);axis([0 inf -inf inf]); grid on; ylabel('roll');
subplot(313);plot(t_imu,e_bno(:,3)*180/pi);hold all; plot(t_jig_comp,-t_angle); plot(t_rec,-t_angle_expect,'r*');axis([0 inf -200 200]); grid on;
ylabel('pitch');

figure; 
subplot(311);plot(t_imu,e_bno(:,1)*180/pi); hold all; plot(t_match,base_match); plot(t_rec,b_angle_expect,'r*'); axis([0 inf -200 200]); grid on;
title('Time matched'); ylabel('yaw'); legend('BNO','Jig','expectation');
subplot(312);plot(t_imu,e_bno(:,2)*180/pi);axis([0 inf -inf inf]); grid on; ylabel('roll');
subplot(313);plot(t_imu,e_bno(:,3)*180/pi);hold all; plot(t_match,-top_match); plot(t_rec,-t_angle_expect,'r*');axis([0 inf -200 200]); grid on;
ylabel('pitch');

figure; plot(t_match,e_diff*180/pi); legend('Yaw','Roll','Pitch'); ylabel('Angle [°]');
xlabel('time [s]'); axis([t_match(1) inf -inf inf]); title('Angle Error');

figure; 
subplot(311);plot(t_imu,e_bno(:,1)*180/pi); hold all; plot(t_match,base_match); plot(t_match,e_diff(:,1)*180/pi); axis([0 inf -200 200]); grid on;
title('Time matched'); ylabel('yaw'); legend('BNO','Jig','error');
subplot(312);plot(t_imu,e_bno(:,2)*180/pi);hold all; plot(t_imu,zeros(size(e_bno(:,2)))); plot(t_match,e_diff(:,2)*180/pi); 
axis([0 inf -inf inf]); grid on; ylabel('roll');
subplot(313);plot(t_imu,e_bno(:,3)*180/pi);hold all; plot(t_match,-top_match); plot(t_match,e_diff(:,3)*180/pi);axis([0 inf -200 200]); grid on;
ylabel('pitch');























% find the new time vector position for the jig position [c index] = min(abs(N-V(1)))
% %convert jig angles to quaternions
% resampling_factor = length(t_comp)/length(t_imu);                   % determining the oversampling factor
% re_Q_norm = resample(Q_norm,round(resampling_factor*10000),10000);  % oversampling the BNO quaternions
% for(i=1:length(t_angle))
%     re_Q_norm(i,:) = re_Q_norm(i,:)/norm(re_Q_norm(i,:));
% end
% re_Q_norm(end,:) = [];
% Q_jig = zeros(length(t_angle),4);
% for(i=1:length(t_angle))
%     Q_jig(i,:) = angle2quat(b_angle(i)*pi/180, 0,  -t_angle(i)*pi/180,'ZXY');
% end
% [e_jig(:,1),e_jig(:,2),e_jig(:,3)] = quat2angle(Q_jig,'ZXY');
% [e_bno2(:,1),e_bno2(:,2),e_bno2(:,3)] = quat2angle(re_Q_norm,'ZXY');
% plot(e_jig); hold on; plot(e_bno2);
