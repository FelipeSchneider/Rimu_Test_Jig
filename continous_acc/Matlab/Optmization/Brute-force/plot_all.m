
load('fit_lsm.mat')
[best_CF_lsm] = min(kte_CF);
[best_madgwick_lsm] = min(kte_madgwick);
[best_mahony_lsm] = min(kte_mahony);

figure(1);
plot(steps_CF,kte_CF); hold all; title('CF');
figure(2);
plot(steps,kte_madgwick); hold all; title('Madgwick');
figure(3);
plot(steps,kte_mahony); hold all; title('Mahony');


load('fit_bno.mat')
[best_CF_bno] = min(kte_CF);
[best_madgwick_bno] = min(kte_madgwick);
[best_mahony_bno] = min(kte_mahony);

figure(1);
plot(steps_CF,kte_CF); hold all;
figure(2);
plot(steps,kte_madgwick); hold all;
figure(3);
plot(steps,kte_mahony); hold all;

load('fit_mean.mat')
[best_CF_mean] = min(kte_CF);
[best_madgwick_mean] = min(kte_madgwick);
[best_mahony_mean] = min(kte_mahony);

figure(1);
plot(steps_CF,kte_CF); hold all;
figure(2);
plot(steps,kte_madgwick); hold all;
figure(3);
plot(steps,kte_mahony); hold all;

load('fit_lsm_acc_bno_giro.mat')
[best_CF_lsm_acc] = min(kte_CF);
[best_madgwick_lsm_acc] = min(kte_madgwick);
[best_mahony_lsm_acc] = min(kte_mahony);

figure(1);
plot(steps_CF,kte_CF); hold all;
figure(2);
plot(steps,kte_madgwick); hold all;
figure(3);
plot(steps,kte_mahony); hold all;

load('fit_lsm_giro_bno_acc.mat')
[best_CF_lsm_giro] = min(kte_CF);
[best_madgwick_lsm_giro] = min(kte_madgwick);
[best_mahony_lsm_giro] = min(kte_mahony);

figure(1);
plot(steps_CF,kte_CF); hold all;
figure(2);
plot(steps,kte_madgwick); hold all;
figure(3);
plot(steps,kte_mahony); hold all;

load('fit_lsm_giro_bno_acc_lis_mag.mat')
[best_CF_lis] = min(kte_CF);
[best_madgwick_lis] = min(kte_madgwick);
[best_mahony_lis] = min(kte_mahony);

figure(1);
plot(steps_CF,kte_CF); hold all; legend('LSM','BNO','LSM & BNO mean','LSM acc BNO gyro&mag','LSM gyro BNO acc&mag','LSM-BNO-LIS');
figure(2);
plot(steps,kte_madgwick); hold all;legend('LSM','BNO','LSM & BNO mean','LSM acc BNO gyro&mag','LSM gyro BNO acc&mag','LSM-BNO-LIS');
xlabel('Beta'); ylabel('KTE')
figure(3);
plot(steps,kte_mahony); hold all;legend('LSM','BNO','LSM & BNO mean','LSM acc BNO gyro&mag','LSM gyro BNO acc&mag','LSM-BNO-LIS');