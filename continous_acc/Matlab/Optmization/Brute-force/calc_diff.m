function [e_diff] = calc_diff(q_out,q_jig)



[e_jig(:,1),e_jig(:,2),e_jig(:,3)] = quat2angle(q_jig,'ZXY');
[e_out(:,1),e_out(:,2),e_out(:,3)] = quat2angle(q_out,'ZXY');
e_diff(:,1) = angdiff(e_jig(:,1),e_out(:,1));
e_diff(:,2) = angdiff(e_jig(:,2),e_out(:,2));
e_diff(:,3) = angdiff(e_jig(:,3),e_out(:,3));
e_diff(isnan(e_diff)) = 0;

e_diff = (e_diff.*180)./pi;
end