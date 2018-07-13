clear all
load('fit_bno.mat') %1
[best_CF(1), i_CF(1)] = min(kte_CF);
[best_madgwick(1), i_madgwick(1)] = min(kte_madgwick);
[best_mahony(1), i_mahony(1)] = min(kte_mahony);
k_CF(1) = steps_CF(i_CF(1));
k_madgwick(1) = steps(i_madgwick(1));
k_mahony(1) = steps(i_mahony(1));
figure(1); plot(steps_CF,kte_CF); hold all; title('CF');
figure(2); plot(steps,kte_madgwick); hold all; title('Madgwick');
figure(3); plot(steps,kte_mahony); hold all; title('Mahony');


load('fit_lsm_lis_mag.mat') %2 errado
[best_CF(2), i_CF(2)] = min(kte_CF);
[best_madgwick(2), i_madgwick(2)] = min(kte_madgwick);
[best_mahony(2), i_mahony(2)] = min(kte_mahony);
k_CF(2) = steps_CF(i_CF(2));
k_madgwick(2) = steps(i_madgwick(2));
k_mahony(2) = steps(i_mahony(2));
figure(1); plot(steps_CF,kte_CF); hold all;
figure(2); plot(steps,kte_madgwick); hold all;
figure(3); plot(steps,kte_mahony); hold all;

load('fit_bno_acc_giro_lis_mag.mat') %3
[best_CF(3), i_CF(3)] = min(kte_CF);
[best_madgwick(3), i_madgwick(3)] = min(kte_madgwick);
[best_mahony(3), i_mahony(3)] = min(kte_mahony);
k_CF(3) = steps_CF(i_CF(3));
k_madgwick(3) = steps(i_madgwick(3));
k_mahony(3) = steps(i_mahony(3));
figure(1); plot(steps_CF,kte_CF); hold all;
figure(2); plot(steps,kte_madgwick); hold all;
figure(3); plot(steps,kte_mahony); hold all;

load('fit_lsm_acc_bno_giro_lis_mag.mat') %4
[best_CF(4), i_CF(4)] = min(kte_CF);
[best_madgwick(4), i_madgwick(4)] = min(kte_madgwick);
[best_mahony(4), i_mahony(4)] = min(kte_mahony);
k_CF(4) = steps_CF(i_CF(4));
k_madgwick(4) = steps(i_madgwick(4));
k_mahony(4) = steps(i_mahony(4));
figure(1); plot(steps_CF,kte_CF,':','Linewidth',2); hold all;
figure(2); plot(steps,kte_madgwick,':','Linewidth',2); hold all;
figure(3); plot(steps,kte_mahony,':','Linewidth',2); hold all;

load('fit_lsm_giro_bno_acc_lis_mag.mat') %5
[best_CF(5), i_CF(5)] = min(kte_CF);
[best_madgwick(5), i_madgwick(5)] = min(kte_madgwick);
[best_mahony(5), i_mahony(5)] = min(kte_mahony);
k_CF(5) = steps_CF(i_CF(5));
k_madgwick(5) = steps(i_madgwick(5));
k_mahony(5) = steps(i_mahony(5));
figure(1); plot(steps_CF,kte_CF,'--','Linewidth',1.5); hold all;
figure(2); plot(steps,kte_madgwick,'--','Linewidth',1.5); hold all;
figure(3); plot(steps,kte_mahony,'--','Linewidth',1.5); hold all;


load('fit_mean_bno_mag.mat') %6
[best_CF(6), i_CF(6)] = min(kte_CF);
[best_madgwick(6), i_madgwick(6)] = min(kte_madgwick);
[best_mahony(6), i_mahony(6)] = min(kte_mahony);
k_CF(6) = steps_CF(i_CF(6));
k_madgwick(6) = steps(i_madgwick(6));
k_mahony(6) = steps(i_mahony(6));
figure(1); plot(steps_CF,kte_CF,'-.','Linewidth',1.5); hold all;
figure(2); plot(steps,kte_madgwick,'-.','Linewidth',1.5); hold all;
figure(3); plot(steps,kte_mahony,'-.','Linewidth',1.5); hold all;

load('fit_mean_imu_mag.mat') %7
[best_CF(7), i_CF(7)] = min(kte_CF);
[best_madgwick(7), i_madgwick(7)] = min(kte_madgwick);
[best_mahony(7), i_mahony(7)] = min(kte_mahony);
k_CF(7) = steps_CF(i_CF(7));
k_madgwick(7) = steps(i_madgwick(7));
k_mahony(7) = steps(i_mahony(7));

figure(1);
plot(steps_CF,kte_CF,'Linewidth',2); hold all; legend('1','2','3','4','5','6','7');
xlabel('Crossover factor'); ylabel('KTE'); grid on;
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

figure(2);
plot(steps,kte_madgwick,'Linewidth',2); hold all;legend('1','2','3','4','5','6','7');
xlabel('Beta'); ylabel('KTE'); grid on;
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

figure(3);
plot(steps,kte_mahony,'Linewidth',2); hold all;legend('1','2','3','4','5','6','7');
xlabel('Kp'); ylabel('KTE'); grid on;
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];