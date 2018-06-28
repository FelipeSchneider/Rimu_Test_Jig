function [fit_kte,fit_rms] = calc_Fit(q_out,q_jig)
%calculate the fit for each particle
% q_out is the set of algorithm quaternion orientation output 
% q_jig is the quaternion orientation (reference) for the jig angles

[e_jig(:,1),e_jig(:,2),e_jig(:,3)] = quat2angle(q_jig,'ZXY');
[e_out(:,1),e_out(:,2),e_out(:,3)] = quat2angle(q_out,'ZXY');
e_diff(:,1) = angdiff(e_jig(:,1),e_out(:,1));
e_diff(:,2) = angdiff(e_jig(:,2),e_out(:,2));
e_diff(:,3) = angdiff(e_jig(:,3),e_out(:,3));
e_diff(isnan(e_diff)) = 0;
fit = sqrt(mean(abs(e_diff*180/pi)).^2 + var(e_diff*180/pi));
fit_kte = mean(fit,2); %choose a function that best describe the fit of the 3 angles
fit2 = rms(e_diff)*180/pi;
fit_rms = rms(fit2);
end