function [fit_kte,fit_rms] = calc_Fit(e_jig,e_out)
%calculate the fit for each particle
% q_out is the set of algorithm Euler angles orientation output 
% e_jig is the Euler angles orientation (reference) for the jig angles


e_diff = angdiff(e_jig,e_out);
e_diff(isnan(e_diff)) = 0;

fit = sqrt(mean(abs(e_diff*180/pi)).^2 + var(e_diff*180/pi));
fit_kte = mean(fit,2); %choose a function that best describe the fit of the 3 angles
fit2 = rms(e_diff*180/pi);
fit_rms = mean(fit2);
end