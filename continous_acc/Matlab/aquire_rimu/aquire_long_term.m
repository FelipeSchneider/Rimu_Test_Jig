

for i=1:96
    i
    run aquireIMU_BNO_4_temp.m
    name = sprintf('data_%d',i)
    save(name,'acc_bno_g', 'acc_imu_g', 'giro_bno_dps', 'giro_imu_dps', 'i',...
        'mag_imu_gaus', 'MICRO_TESLA_XY_MAX', 'MICRO_TESLA_Z_MAX', 'time_sample',...
        'temperature_C', 'G_MAX_BNO','G_MAX_IMU','GAUSS_MAX_IMU',...
       'DPS_MAX_BNO','t','Q');
   pause(5);
   close all
   clc
end

figure(1);
subplot(311);plot(t,[giro_imu_dps(1,:); giro_bno_dps(1,:)]'); grid on;
title('Giroscópio');ylabel('X [dps]');legend('Minimu','BNO','Location','northwest','Orientation','horizontal')
subplot(312);plot(t,[giro_imu_dps(2,:); giro_bno_dps(2,:)]'); ylabel('Y [dps]'); grid on;
subplot(313);plot(t,[giro_imu_dps(3,:); giro_bno_dps(3,:)]'); ylabel('Z [dps]'); grid on;
xlabel('Tempo [s]')

figure(2);
subplot(311);plot(t,[acc_imu_g(1,:); acc_bno_g(1,:)]'); grid on;
title('Acelerômetro');ylabel('X [g]');legend('Minimu','BNO','Location','northwest','Orientation','horizontal')
subplot(312);plot(t,[acc_imu_g(2,:); acc_bno_g(2,:)]'); ylabel('Y [g]'); grid on;
subplot(313);plot(t,[acc_imu_g(3,:); acc_bno_g(3,:)]'); ylabel('Z [g]'); grid on;
xlabel('Tempo [s]')