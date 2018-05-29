function [fit] = Kalman_fit(q_out,q_jig)
%calculate the fit for each particle
% q_out is the set of quaternion orientation for all particles
% q_jig is the quaternion orientation (reference) for the jig angles
c = size(q_out,3);
fit = zeros(c,1);
for k=1:c
    q_diff = quatmultiply(quatconj(q_out(:,:,k)),q_jig);
    [e_diff(:,1),e_diff(:,2),e_diff(:,3)] = quat2angle(q_diff,'ZXY');
    fit(k) = norm(e_diff);
end