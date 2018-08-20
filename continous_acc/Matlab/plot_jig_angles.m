load slow_range_data2.mat   %o 2 parece bom
t_slow = t;
base_slow = real_base_angle;
top_slow = real_top_angle;
b_speed_slow = diff(base_slow)/(t(2)-t(1));
t_speed_slow = diff(top_slow)/(t(2)-t(1));

load mid_range_data2.mat   %o 6 ficou com erro baixo
t_mid = t;
base_mid = real_base_angle;
top_mid = real_top_angle;
b_speed_mid = diff(base_mid)/(t(2)-t(1));
t_speed_mid = diff(top_mid)/(t(2)-t(1));

load fast_range_data.mat
t_fast = t;
base_fast = real_base_angle;
top_fast = real_top_angle;
b_speed_fast = diff(base_fast)/(t(2)-t(1));
t_speed_fast = diff(top_fast)/(t(2)-t(1));

clearvars -except t_fast base_fast top_fast b_speed_fast t_speed_fast ...
    t_mid base_mid top_mid b_speed_mid t_speed_mid ...
    t_slow base_slow top_slow b_speed_slow t_speed_slow

%plotar o padrão de velocidades também

m_slow_speed = mean([abs(b_speed_slow),abs(t_speed_slow)])
m_mid_speed = mean([abs(b_speed_mid),abs(t_speed_mid)])
m_fast_speed = mean([abs(b_speed_fast),abs(t_speed_fast)])

figure;
p = panel();
p.pack('v',2);
p(1).pack({1}, {100});
p(2).pack({1}, {100});
p.de.margin = 4;
p.margin = [15 15 5 10];
p.select('all');
p.fontsize = 10;
p.identify();

p(1,1,1).select();
plot(t_slow, base_slow,'Linewidth',1.3); hold all;
plot(t_mid, base_mid,'Linewidth',1.3);
plot(t_fast, base_fast,'Linewidth',1.3);
axis([0 inf -inf inf]);
grid on; title('Fusion comparison'); ylabel('Yaw orientation [°]');
legend('Slow','Mid','Fast','Orientation','Horizontal');
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(t_slow, top_slow,'Linewidth',1.3); hold all;
plot(t_mid, top_mid,'Linewidth',1.3);
plot(t_fast, top_fast,'Linewidth',1.3);
axis([0 inf -inf inf]); grid on; 
ylabel('Pitch orientation [°]'); xlabel('Time [s]');
p(2).marginbottom = 10;

figure;
p = panel();
p.pack('v',2);
p(1).pack({1}, {100});
p(2).pack({1}, {100});
p.de.margin = 4;
p.margin = [15 15 5 10];
p.select('all');
p.fontsize = 10;
p.identify();

p(1,1,1).select();
plot(t_slow(1:end-1), b_speed_slow,'Linewidth',1.3); hold all;
plot(t_mid(1:end-1), b_speed_mid,'Linewidth',1.3);
plot(t_fast(1:end-1), b_speed_fast,'Linewidth',1.3);
axis([0 inf -inf inf]);
grid on; title('Fusion comparison'); ylabel('Yaw speed [°/s]');
legend('Slow','Mid','Fast','Orientation','Horizontal');
a = gca; a.XTickLabel = {};

p(2,1,1).select();
plot(t_slow(1:end-1), t_speed_slow,'Linewidth',1.3); hold all;
plot(t_mid(1:end-1), t_speed_mid,'Linewidth',1.3);
plot(t_fast(1:end-1), t_speed_fast,'Linewidth',1.3);
axis([0 inf -inf inf]); grid on; 
ylabel('Pitch speed [°/s]'); xlabel('Time [s]');
p(2).marginbottom = 10;