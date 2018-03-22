function [ j,giro_imu, acc_imu, mag_imu, giro_bno, acc_bno, mag_bno, Q ]...
    = readDataImu(j, ACTION, s_imu, giro_imu, acc_imu, mag_imu, giro_bno, acc_bno, mag_bno, Q)
%[ j,giro_imu, acc_imu, mag_imu, giro_bno, acc_bno, mag_bno, Q ] = readDataImu(j, ACTION, s_imu,giro_imu, acc_imu, mag_imu, giro_bno, acc_bno, mag_bno, Q)
%   read data from the imu acording to the type of action that was sent to
%   the imu.
%   j       the actual vector position of the data that 
switch ACTION
    case 160    %0xA0// acquire raw data from LSM at 500Hz 
        fs = 500;
        imu_bytes = s_imu.BytesAvailable;
        if((imu_bytes >= 12))
            giro_imu(:,j) = fread(s_imu,3,'int16');
            acc_imu(:,j) = fread(s_imu,3,'int16');
            j = j+1;
            if(mod(j,fs)==0)                    %print o segundo atual
                fprintf('Second number %d \r',j/fs)
            end
        end
    case 161    %0xA1// acquire raw data from LIS at 500Hz
        fs = 500;
        imu_bytes = s_imu.BytesAvailable;
        if((imu_bytes >= 6))
            mag_imu(:,j) = fread(s_imu,3,'int16');
            j = j+1;
            if(mod(j,fs)==0)                    %print o segundo atual
                fprintf('Second number %d \r',j/fs)
            end
        end
    case 162    %0xA2// acquire raw data from LSM and LIS at 500Hz
        fs = 500;
        imu_bytes = s_imu.BytesAvailable;
        if((imu_bytes >= 18))
            giro_imu(:,j) = fread(s_imu,3,'int16');
            acc_imu(:,j) = fread(s_imu,3,'int16');
            mag_imu(:,j) = fread(s_imu,3,'int16');
            j = j+1;
            if(mod(j,fs)==0)                    %print o segundo atual
                fprintf('Second number %d \r',j/fs)
            end
        end
    case 163    %0xA3// acquire raw data from BNO055 at 100Hz
        fs = 100;
        imu_bytes = s_imu.BytesAvailable;
        if((imu_bytes >= 18))
            acc_bno(:,j) = fread(s_imu,3,'int16');
            mag_bno(:,j) = fread(s_imu,3,'int16');
            giro_bno(:,j) = fread(s_imu,3,'int16');
            j = j+1;
            if(mod(j,fs)==0)                    %print o segundo atual
                fprintf('Second number %d \r',j/fs)
            end
        end
    case 164    %0xA4 // acquire raw data from LSM, LIS and BNO at 100Hz
        fs = 100;
        imu_bytes = s_imu.BytesAvailable;
        if((imu_bytes >= 36))
            giro_imu(:,j) = fread(s_imu,3,'int16');
            acc_imu(:,j) = fread(s_imu,3,'int16');
            mag_imu(:,j) = fread(s_imu,3,'int16');

            acc_bno(:,j) = fread(s_imu,3,'int16');
            mag_bno(:,j) = fread(s_imu,3,'int16');
            giro_bno(:,j) = fread(s_imu,3,'int16');
            j = j+1;
            if(mod(j,fs)==0)                    %print o segundo atual
                fprintf('Second number %d \r',j/fs)
            end
        end
    case 165    %0xA5// acquire raw data from LSM, LIS and BNO and fusion data in quaternion format from BNO at 100Hz
        fs = 100;
        imu_bytes = s_imu.BytesAvailable;
        if((imu_bytes >= 44))
            giro_imu(:,j) = fread(s_imu,3,'int16');
            acc_imu(:,j) = fread(s_imu,3,'int16');
            mag_imu(:,j) = fread(s_imu,3,'int16');

            acc_bno(:,j) = fread(s_imu,3,'int16');
            mag_bno(:,j) = fread(s_imu,3,'int16');
            giro_bno(:,j) = fread(s_imu,3,'int16');

            Q(:,j) = fread(s_imu,4,'int16');
            j = j+1;
            if(mod(j,fs)==0)                    %print o segundo atual
                fprintf('Second number %d \r',j/fs)
            end
        end
    case 166    %0xA6// acquire raw data from LSM at 500Hz, LIS 100Hz and BNO 100Hz and fusion data in quaternion format from BNO at 100Hz
        error('The acquisition of raw and fusioned data with LSM at 500Hz is not working well')
    case 167    %0xA7// acquire raw data from LSM at 1khz and Lis at 100Hz. No fusion

    case 168    %0xA8// acquire raw data from LSM and LIS at 500Hz but only start when the PA1
        fs = 500;
        imu_bytes = s_imu.BytesAvailable;
        if((imu_bytes >= 18))
            giro_imu(:,j) = fread(s_imu,3,'int16');
            acc_imu(:,j) = fread(s_imu,3,'int16');
            mag_imu(:,j) = fread(s_imu,3,'int16');
            j = j+1;
            if(mod(j,fs)==0)                    %print o segundo atual
                fprintf('Second number %d \r',j/fs)
            end
        end
    case 169    %0xA9//same as ACT_AQR_BOTH_RAW_FUSION_100HZ  but only start when the PA1
        fs = 100;
        imu_bytes = s_imu.BytesAvailable;
        if((imu_bytes >= 44))
            giro_imu(:,j) = fread(s_imu,3,'int16');
            acc_imu(:,j) = fread(s_imu,3,'int16');
            mag_imu(:,j) = fread(s_imu,3,'int16');

            acc_bno(:,j) = fread(s_imu,3,'int16');
            mag_bno(:,j) = fread(s_imu,3,'int16');
            giro_bno(:,j) = fread(s_imu,3,'int16');

            Q(:,j) = fread(s_imu,4,'int16');
            j = j+1;
            if(mod(j,fs)==0)                    %print o segundo atual
                fprintf('Second number %d \r',j/fs)
            end
        end
    otherwise
        error('The action is not implemented yet');
end

end

