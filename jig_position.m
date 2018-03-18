run('gig_speed.m');

ACC_MOD = 1;  %every time that the interruption happens, we will speed up .25 degrees per second
                %we must remember that the possible speeds are int
                %variable.
BREAKING_MOD = 1;

d_theta = 360/(FS_REVOLUTION*DEFAULT_MS);
real_period = (1e-6 * real_period); real_period(1) = inf;        % now in microseconds
dw = d_theta./real_period;

apply_period = repelem(real_period,1/ACC_MOD);
apply_w = d_theta./apply_period;
apply_acc = diff(apply_w,1); apply_acc = apply_acc(1/ACC_MOD:1/ACC_MOD:end);
apply_position = cumsum(apply_w);

figure;
plot(apply_acc); axis([0 inf 0 2]); xlabel('speed'); ylabel('acc mod');


%% a second manner to think:

apply_period(1:4) = 0;
s_step_t = cumsum(apply_period); %singular step time, the time that each step happened
ang_position = (1:length(s_step_t))*d_theta;
d_angle = diff(ang_position);       d_angle(numel(ang_position)) = 0;
dt = diff(s_step_t);                dt(numel(dt)+1:numel(ang_position)) = 1;
ang_vel = d_angle./dt;
d_vel = diff(ang_vel);       d_vel(numel(ang_position)) = 0;
ang_acc = d_vel./dt;
figure; plot(s_step_t,ang_position); xlabel('time (s)'); ylabel('ang position (degrees)');
title('Ang Pos x time');
figure; plot(s_step_t,ang_vel); xlabel('time (s)'); ylabel('ang vel (degrees/s)');
title('Ang Vel x time');
ang_acc2 = ang_acc(3:1/ACC_MOD:end);
ang_acc2 = ang_acc2*ACC_MOD;
figure; plot(ang_acc); xlabel('time (s)'); ylabel('ang acc (degrees/s^2)');
title('Ang Acc x time');


%% a third manner to think
dt = (1:length(apply_period))*0.01;
w = apply_w;
dw = diff(w); dw(numel(dt)) = 0;
theta = cumsum(w)*d_theta*0.01;
ang_acc = dw./0.01;             
ang_acc3 = ang_acc(4:1/ACC_MOD:end);


