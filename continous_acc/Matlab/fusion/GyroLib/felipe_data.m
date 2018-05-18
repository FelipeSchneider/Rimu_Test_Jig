[Euler_, bw_, Angles_] = test_ahrs_quat_data(cumsum(giro_bno_dps*2*pi/360)'*(2*pi/360), ...
    sacc_bno_g', mag_imu_gaus_cal', [0.0336    0.0061    0.9994]', [0.0191   -0.4127    0.9107]', 0.01);




[Euler_, bw_, Angles_] = test_ahrs_quat_data(cumsum(giro_imu_dps*2*pi/360)'*(2*pi/360), ...
    acc_imu_g', mag_imu_gaus_cal', [0.0336    0.0061    0.9994]', [0.0191   -0.4127    0.9107]', 0.01);

[Euler_, bw_, Angles_] = test_ahrs_quat_data((giro_imu_dps*2*pi/360)'*(pi/360),...
    acc_imu_g', mag_imu_gaus_cal', [0.0102   -0.0083    0.9999]', [-0.5676    0.5591    0.6044]', 0.01);


[Euler_, bw_, Angles_] = test_ahrs_quat_data((giro_imu_dps*2*pi/360)'*(0.01),...
    acc_imu_g', mag_imu_gaus_cal', [0.0102   -0.0083    0.9999]', [-0.5676    0.5591    0.6044]', 0.01);



mean(acc_imu_g(:,100:400)');
mean(mag_bno_gaus(:,100:400)');

[Euler_, bw_, Angles_, q_] = test_ahrs_quat_data((giro_imu_dps*pi/180)'*(0.01),...
    acc_imu_g', mag_bno_gaus', [0.0029    0.0170    0.9999]', [0.0490    0.7227    0.5715]', 0.01);