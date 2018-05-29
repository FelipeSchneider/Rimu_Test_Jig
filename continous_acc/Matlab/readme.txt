com_jig_imu.m
This script is similar to the "com_jig.m" present in the jig subfolder.

However, this script also communicates with the IMU through a Bluetooth link, 
acquiring its data according to the protocol (now it is sampling at 100Hz, both IMUs
[LSM, LIS and BNO], acquiring raw and fusioned data).

