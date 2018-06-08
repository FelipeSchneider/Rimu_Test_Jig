function [fit] = Kalman_fit(q_out,q_jig)
%calculate the fit for each particle
% q_out is the set of quaternion orientation for all particles
% q_jig is the quaternion orientation (reference) for the jig angles
c = size(q_out,3);
fit = zeros(c,3);

[e_jig(:,1),e_jig(:,2),e_jig(:,3)] = quat2angle(q_jig,'ZXY');
e_jig(:,3) = -e_jig(:,3);
% q_jig = angle2quat(e_jig(:,1), e_jig(:,2), e_jig(:,3),'ZXY');
% e_diff(isnan(e_diff_bno)) = 0;

for k=1:c
%     q_diff = quatmultiply(quatconj(q_out(:,:,k)),q_jig);
%     [e_diff(:,1),e_diff(:,2),e_diff(:,3)] = quat2angle(q_diff,'ZXY');
%     fit(k) = norm(e_diff);
    [e_kalman(:,1),e_kalman(:,2),e_kalman(:,3)] = quat2angle(q_out(:,:,k),'ZXY');
    e_diff(:,1) = angdiff(abs(e_jig(:,1)),abs(e_kalman(:,1)));
    e_diff(:,2) = angdiff(e_jig(:,2),e_kalman(:,2));
    e_diff(:,3) = angdiff(e_jig(:,3),e_kalman(:,3));
    e_diff(isnan(e_diff)) = 0;
    fit(k,:) = rms(e_diff)*180/pi;
    
    figure(15); 
    subplot(311);plot(e_jig(:,1));hold on;plot(e_kalman(:,1));%plot(e_diff(:,1));
    ylabel('yaw [rad]'); grid on;
    subplot(312);plot(e_jig(:,2));hold on;plot(e_kalman(:,2));%plot(e_diff(:,2));
    ylabel('roll [rad]'); grid on;
    subplot(313);plot(e_jig(:,3));hold on;plot(e_kalman(:,3));%plot(e_diff(:,3));
    ylabel('pitch [rad]'); grid on;
end