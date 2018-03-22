function [t_imu, fs, n_sample, giro_imu, acc_imu, mag_imu, giro_bno, acc_bno, mag_bno, Q]...
    = prelocateRimuVar( ACTION, time_sample )
%   [t, giro_imu, acc_imu, mag_imu, giro_bno, acc_bno, mag_bno, Q] = prelocateRimuVar( ACTION )
%   Prelocate the variables according to the action and the time that will
%   be sampled. The atcion is determined by the RIMU protocol, which consists
%   of basically:
% typedef enum{
% 	NOTHING								= 0x00,
% 	
% 	ACT_AQR_LSM_500HZ					= 0xA0, // acquire raw data from LSM at 500Hz     -- 160 in decimal  
% 	ACT_AQR_LIS_500HZ					= 0xA1, // acquire raw data from LIS at 500Hz
% 	ACT_AQR_IMU_500HZ					= 0xA2, // acquire raw data from LSM and LIS at 500Hz
% 	ACT_AQR_BNO_100HZ					= 0xA3, // acquire raw data from BNO055 at 500Hz
% 	ACT_AQR_BOTH_RAW_100HZ				= 0xA4, // acquire raw data from LSM, LIS and BNO at 100Hz
% 	ACT_AQR_BOTH_RAW_FUSION_100HZ		= 0xA5, // acquire raw data from LSM, LIS and BNO and fusion data in quaternion format from BNO at 100Hz
% 	ACT_AQR_BOTH_RAW_FUSION_500_100HZ	= 0xA6, // acquire raw data from LSM at 500Hz, LIS 100Hz and BNO 100Hz and fusion data in quaternion format from BNO at 100Hz
% 	ACT_AQR_IMU_1KHZ					= 0xA7,	// acquire raw data from LSM at 1khz and Lis at 100Hz. No fusion
% 	ACT_AQR_IMU_500HZ_WAIT				= 0xA8, // acquire raw data from LSM and LIS at 500Hz but only start when the PA1
% 	ACT_AQR_BOTH_RAW_FUSION_100HZ_WAIT  = 0xA9, //same as ACT_AQR_BOTH_RAW_FUSION_100HZ  but only start when the PA1
% 	
% 	ACT_BNO_STATUS						= 0xB0, // ask all the BNO status (the order of the answer is describe in the matlab document)
% 	ACT_READ_CALIBRATION				= 0xB1, // read all calibration vectors
% 	ACT_TRIGGER_AUTO_TEST				= 0xB2, // trigger auto test for BNO
% 	
% 	ACT_TUG_100HZ						= 0xC0, // Acquire for the TUG test, this will acquire at 100Hz raw data from ACC and Gyro from both sensors, quaternions and Linear acc from BNO
% 	ACT_TUG_100HZ_WAIT					= 0xC1, // same as ACT_TUG_100HZ but wait for synchronization
% 	ACT_STOP							= 0xEE, // stop acquisition
% }ACTION_TYPE;

switch ACTION
    case 160    %0xA0// acquire raw data from LSM at 500Hz 
        fs = 500;
        n_sample = time_sample*fs;
        t_imu = 0:1/fs:time_sample-1/fs; %RIMU time vector
        giro_imu = zeros(3,n_sample); %performance improvement with prellocating
        acc_imu = zeros(3,n_sample); 
        mag_imu = []; 

        giro_bno = []; 
        acc_bno = []; 
        mag_bno = [];
        Q = [];
        disp('RIMU: acquire raw data from LSM at 500Hz')
    case 161    %0xA1// acquire raw data from LIS at 500Hz
        fs = 500;
        n_sample = time_sample*fs;
        t_imu = 0:1/fs:time_sample-1/fs; %RIMU time vector
        giro_imu = []; %performance improvement with prellocating
        acc_imu = []; 
        mag_imu = zeros(3,n_sample);

        giro_bno = []; 
        acc_bno = []; 
        mag_bno = [];
        Q = [];
        disp('RIMU: acquire raw data from LIS at 500Hz')
    case 162    %0xA2// acquire raw data from LSM and LIS at 500Hz
        fs = 500;
        n_sample = time_sample*fs;
        t_imu = 0:1/fs:time_sample-1/fs; %RIMU time vector
        giro_imu = zeros(3,n_sample); %performance improvement with prellocating
        acc_imu = zeros(3,n_sample); 
        mag_imu = zeros(3,n_sample); 

        giro_bno = []; 
        acc_bno = []; 
        mag_bno = [];
        Q = [];
        disp('RIMU: acquire raw data from LSM and LIS at 500Hz')
    case 163    %0xA3// acquire raw data from BNO055 at 100Hz
        fs = 100;
        n_sample = time_sample*fs;
        t_imu = 0:1/fs:time_sample-1/fs; %RIMU time vector
        giro_imu = zeros(3,n_sample); %performance improvement with prellocating
        acc_imu = zeros(3,n_sample); 
        mag_imu = zeros(3,n_sample); 

        giro_bno = zeros(3,n_sample); 
        acc_bno = zeros(3,n_sample); 
        mag_bno = zeros(3,n_sample);
        Q = zeros(4,n_sample);
        disp('RIMU: acquire raw data from BNO055 at 100Hz')
    case 164    %0xA4 // acquire raw data from LSM, LIS and BNO at 100Hz
        fs = 100;
        n_sample = time_sample*fs;
        t_imu = 0:1/fs:time_sample-1/fs; %RIMU time vector
        giro_imu = zeros(3,n_sample); %performance improvement with prellocating
        acc_imu = zeros(3,n_sample); 
        mag_imu = zeros(3,n_sample); 

        giro_bno = zeros(3,n_sample); 
        acc_bno = zeros(3,n_sample); 
        mag_bno = zeros(3,n_sample);
        Q = [];
        disp('RIMU: acquire raw data from LSM, LIS and BNO at 100Hz')
    case 165    %0xA5// acquire raw data from LSM, LIS and BNO and fusion data in quaternion format from BNO at 100Hz
        fs = 100;
        n_sample = time_sample*fs;
        t_imu = 0:1/fs:time_sample-1/fs; %RIMU time vector
        giro_imu = zeros(3,n_sample); %performance improvement with prellocating
        acc_imu = zeros(3,n_sample); 
        mag_imu = zeros(3,n_sample); 

        giro_bno = zeros(3,n_sample); 
        acc_bno = zeros(3,n_sample); 
        mag_bno = zeros(3,n_sample);
        Q = zeros(4,n_sample);
        disp('RIMU: acquire raw data from LSM, LIS and BNO and fusion data in quaternion format from BNO at 100Hz')
    case 166    %0xA6// acquire raw data from LSM at 500Hz, LIS 100Hz and BNO 100Hz and fusion data in quaternion format from BNO at 100Hz
        error('The acquisition of raw and fusioned data with LSM at 500Hz is not working well')
    case 167    %0xA7// acquire raw data from LSM at 1khz and Lis at 100Hz. No fusion
        error('not yet implemented')
%         fs = 100;
%         n_sample = time_sample*fs;
%         t_imu = 0:1/fs:time_sample-1/fs; %RIMU time vector
%         giro_imu = zeros(3,n_sample); %performance improvement with prellocating
%         acc_imu = zeros(3,n_sample); 
%         mag_imu = zeros(3,n_sample); 
% 
%         giro_bno = []; 
%         acc_bno = []; 
%         mag_bno = [];
%         Q = [];
%         disp('RIMU: acquire raw data from LSM, LIS and BNO at 100Hz')
    case 168    %0xA8// acquire raw data from LSM and LIS at 500Hz but only start when the PA1
        fs = 500;
        n_sample = time_sample*fs;
        t_imu = 0:1/fs:time_sample-1/fs; %RIMU time vector
        giro_imu = zeros(3,n_sample); %performance improvement with prellocating
        acc_imu = zeros(3,n_sample); 
        mag_imu = zeros(3,n_sample); 

        giro_bno = []; 
        acc_bno = []; 
        mag_bno = [];
        Q = [];
        disp('RIMU: acquire raw data from LSM and LIS at 500Hz but only start when the PA1')
    case 169    %0xA9//same as ACT_AQR_BOTH_RAW_FUSION_100HZ  but only start when the PA1
        fs = 100;
        n_sample = time_sample*fs;
        t_imu = 0:1/fs:time_sample-1/fs; %RIMU time vector
        giro_imu = zeros(3,n_sample); %performance improvement with prellocating
        acc_imu = zeros(3,n_sample); 
        mag_imu = zeros(3,n_sample); 

        giro_bno = zeros(3,n_sample); 
        acc_bno = zeros(3,n_sample); 
        mag_bno = zeros(3,n_sample);
        Q = zeros(4,n_sample);
        disp('RIMU: cquire raw data from LSM, LIS and BNO and fusion data in quaternion format from BNO at 100Hz but only start when the PA1')
    otherwise
        error('The action is not implemented yet');
end
end

