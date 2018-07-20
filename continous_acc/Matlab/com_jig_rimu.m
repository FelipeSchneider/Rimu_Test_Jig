clear all; %clc; 
close all;
if(~isempty(instrfind))
    disp('Closing all COM ports')
    fclose(instrfind);
end

addpath('fusion');
addpath('jig');
addpath('aquire_rimu');
%% angles and speed configurations - user configurable up to 249 commands for each motor

%mid range test
base_angle = [5000 30  30   30   1000  130  30  40  80   2000   45  45   75  2000 150  35  90  300  120 45 3000  60 120 200 45  150  15   30   1000  85  45 45  45  95 60  30  30   3000];
base_speed = [0    60  100  150  0    -110  45  -80 200  0      300 -300 60  0    -60  99  145 -120 200 64 0     89 254 -99 130 -89 -78  -60   0     320 60 -90 120 72 123 -99 -210 0];
top_angle = [5000  45   99  120  1000 120   15  45  240  2000   75  360  65  2000 200  99  46  135  120 90 1000  20 90  170 25  20   95   30   1000 145 115 260  60 60  360  60 124  3000];
top_speed = [0     200  120 -45  0    -75   45  -90 -200 0      200 -85  160 0    90  -99  145 -120 220 64 0     89 154 -99 30  -89 -220 -70   0    120 187 -180 60 -30 104  55  109 0];
time_sample = 70;      %number of seconds that we will collect

%fast speed
% base_angle = [5000 90  120  50   150  30  180 180  30    60   360 450   90   150  145 25   35   90  78  14   223  165 245 129  250  25   25  140  257 142  15   25   35  165  85   195  5000];
% base_speed = [0    200 300  -100 -200 115 320 -180 100   -245 220 -250 -130 -150  60  78  -145 -75 -98 -125  -145 120 220 -150 124  -112 325 -354 250 315  152  65   104 -200 115 -114  0];
% top_angle = [5000  85  165  35   25   15  142 360  409   25   25  25   250   120  245 165  120  15  78  90   35   25  175  145 150  90   80  60   60  30   180  180  150  50  120  90   5000];
% top_speed = [0     410 -500 184  265  152 150 250 -154   -300 152 250  224  -210  150 320 -145 -125 -98 -75 -145  278 265  100 -150 -130 95  -100 220 -245 100  -180 115 -250 -114 -300 0];
% time_sample = 70;      %number of seconds that we will collect

%slow speed
% base_angle = [5000	10	10	10	10	10	10	90	2000	90	2000	90	2000	30	20	1000	30	1000	30	1000	30	1000	90	1000	30	1000	15	500	15	500	0	15	20	15	35	15	22	1500	120	1500	10	30	10	1000	10	10	1000	10	1000	10	1000	10	1000	20	77	5000];
% base_speed = [0 0 0 0 0 0 0 70 0 70 0 70 0 -100 -90 0 -100 0 -100 0 -150 0 -150 0 -150 0 200 0 200 0 0 200 300 -200 0 -200 -120 0 -130 0 -100 100 -100 0 -100 -100 0 -100 0 100 0 100 0 90 90 0];
% top_angle = [5000	90	2000	90	2000	90	2000	10	10	10	10	10	20	30	1000	30	1000	30	1000	30	1000	30	1000	15	500	15	500	15	500	15	500	15	35	1500	45	1500	120	1500	10	20	10	1500	10	1500	10	1500	10	1500	10	1500	10	1500	10	10	125	5000];
% top_speed = [0	70	0	70	0	70	0	0	0	0	0	0	100	-100	0	-100	0	-100	0	-100	0	-150	0	200	0	200	0	200	0	200	0	200	-120	0	-130	0	-100	0	100	75	100	0	100	0	100	0	100	0	100	0	100	0	100	100	-100	0];
% time_sample = 75;      %number of seconds that we will collect

%% Yaw correction 
% base_angle = [10 5000];% move 10 degrees, wait 5000ms
% base_speed = [60    0];% move at 60 degrees per second, command to wait
% top_angle = [0 0];%wait 2000ms, move 360 degrees
% top_speed = [0 0]; %command to wait (wait for the base move), move at 200dps
% j=0;
% for i=20:10:360
%     j=j+1;
%    base_angle = [base_angle [10 5000]];
%    base_speed = [base_speed [60 0]];
%    top_angle = [top_angle [0 0]];
%    top_speed = [top_speed [0 0]];
% end
% top_angle = [5000 top_angle];
% top_speed = [0 top_speed];
% base_angle = [5000 base_angle];
% base_speed = [0 base_speed];
% time_sample = 200;      %number of seconds that we will collect
%% Pitch correction 
% base_angle = [0 0];% move 10 degrees, wait 5000ms
% base_speed = [0    0];% move at 60 degrees per second, command to wait
% top_angle = [10 5000];%wait 2000ms, move 360 degrees
% top_speed = [60 0]; %command to wait (wait for the base move), move at 200dps
% j=0;
% for i=20:10:360
%     j=j+1;
%    base_angle = [base_angle [0 0]];
%    base_speed = [base_speed [0 0]];
%    top_angle = [top_angle [10 5000]];
%    top_speed = [top_speed [60 0]];
% end
% top_angle = [5000 top_angle];
% top_speed = [0 top_speed];
% base_angle = [5000 base_angle];
% base_speed = [0 base_speed];
% time_sample = 200;      %number of seconds that we will collect

%% rimu definitions
BLUETOOTH = 1;          %to use bluetooth set this flag
COM_imu = 16;            %com port number
COM_imu_baud = 230400;   %Baud rate
COM_header_1 = 83;       %Header rimu 1
COM_header_2 = 172;      %Header rimu 2
ACTION = 165;            %read LSM + LIS + BNO + BNO quaternions
%ACTION = 162;

%sensor configurations -- BNO might change this automatically if the fusion
%mode is on
G_MAX_IMU = 8;
G_MAX_BNO = 8;
DPS_MAX_IMU = 1000;
DPS_MAX_BNO = 2000;
GAUSS_MAX_IMU = 4;
MICRO_TESLA_MAX = 40;
COM_jig_num = 11;
%% constants
jig_const = struct('microstep', 16, 'step_per_rev', 200,... 
            'd_theta', 360/(16*200),'min_w',10,'max_w',720,...
            'max_base_angle',720,'acc_mod',360,'breaking_mod',360,...
            'positive_speed','CCW','negative_speed','CW',...
            'limit_com_size',249);
        
%% Jig calculations 
disp('Calculating the commands');
[com_vector, b_angles, t_angles] = createComVector(base_angle, base_speed, top_angle, top_speed ,jig_const);
com_vector2 = typecast(int32(com_vector), 'uint16');
com_vector2 = com_vector2(1:2:end); %taking only the odd positions

[t, real_base_angle, real_top_angle, com_data ] = reconstructCinematic( com_vector, jig_const);
figure(1);
subplot(211);plot(t,real_base_angle); hold on;
plot(com_data(1,:),com_data(2,:),'*');
title('Jig angles'); ylabel('Base angles (°)'); 
subplot(212);plot(t,real_top_angle); hold on;
plot(com_data(1,:),com_data(3,:),'*');
xlabel('Time [s]'); ylabel('Top angles (°)');

% figure(2);
% subplot(211); plot(repelem(b_angles,5)); title('Ilustrative jig angles');
% ylabel('Base angles (°)'); 
% subplot(212);
% plot(repelem(t_angles,5)); ylabel('Top angles (°)'); xlabel('commands sequence');

%% Rimu start of communication
COM_imu_name = sprintf('COM%d',COM_imu);
[t_imu, fs, n_sample, giro_imu, acc_imu, mag_imu, giro_bno, acc_bno, mag_bno, Q]...
    = prelocateRimuVar( ACTION, time_sample );

if(BLUETOOTH == 0)
    disp('Init. Serial-USB conversor')
    s_imu = serial(COM_imu_name,'BaudRate',COM_imu_baud,'DataBits',8);
else
    disp('Init. Bluetooth')
    s_imu = Bluetooth('RIMU',1);
    disp('Conecting')
end
s_imu.InputBufferSize = n_sample*20;
fopen(s_imu);
flushinput(s_imu);
pause(1);

%% Start of communication 
%  the board will answer with an 'S' the start of a set of commands, 
%  with an 'N' every new command and with an 'E' the end of the set of commands
header_start_jig = 'St';
header_command_jig = 'Sc';
COM_jig_name = sprintf('COM%d',COM_jig_num);
COM_jig_baud = 57600;
s_jig = serial(COM_jig_name,'BaudRate',COM_jig_baud,'DataBits',8);
s_jig.InputBufferSize = 2^16;
s_jig.OutputBufferSize = 2^14;
disp('Opening the Jig COM port')
fopen(s_jig);
disp('Waiting for arduino reset')
pause(3);       %waiting arduino to reset
flushinput(s_jig);

disp('Sending the Jig position commands')
fwrite(s_jig,uint8(header_command_jig),'uint8');
fwrite(s_jig,com_vector2,'uint16');

c_exit = 0;
press = 0;
disp('iniciando a coleta')   

time_sample_hi = floor(time_sample/255);
time_sample_lo = time_sample-time_sample_hi*255;

msg = [COM_header_1 COM_header_2 ACTION time_sample_hi time_sample_lo];
fwrite(s_imu,msg,'uint8');
disp('Rimu initial command sent')


disp('Sendind the Jig start command')
fwrite(s_jig,uint8(header_start_jig),'uint8');


%% Receiving 
jig_rec = 0;
jig_bytes = 0; imu_bytes = 0;
t_rec = 0;
j=1;
tic
while((jig_rec ~= 'E')||(j<n_sample))
    jig_bytes = s_jig.BytesAvailable;
    if(jig_bytes > 0)
        jig_rec = fread(s_jig,1,'uint8');  %blower positivo
        t_rec = [t_rec toc];
        if(jig_rec == 'S')
            disp('Start of the set of commands, time:')
            disp(t_rec(end))
        elseif(jig_rec == 'N')
            disp('New command, time:')
            disp(t_rec(end))
        elseif(jig_rec == 'E')
            disp('End of the set of commands, time:')
            disp(t_rec(end))
        else
            disp('Byte received not recognize, time:')
            disp(t_rec(end))
        end
    end
    
    if(j<=n_sample)
        [ j,giro_imu, acc_imu, mag_imu, giro_bno, acc_bno, mag_bno, Q ]...
    = readDataImu(j, ACTION, s_imu, giro_imu, acc_imu, mag_imu, giro_bno, acc_bno, mag_bno, Q);  
    end
    
end
fclose(s_jig);
fclose(s_imu);

%% pos processing
%LSM+LIS
if(~isempty(giro_imu)) 
    giro_imu_dps = double(giro_imu)/(2^15)*DPS_MAX_IMU;
else
    giro_imu_dps = zeros(3,n_sample);
end
if(~isempty(acc_imu)) 
    acc_imu_g = double(acc_imu)/(2^15)*G_MAX_IMU;
else
    acc_imu_g = zeros(3,n_sample);
end
if(~isempty(mag_imu)) 
    mag_imu_gaus = double(mag_imu)/(2^15)*MICRO_TESLA_MAX;
else
    mag_imu_gaus = zeros(3,n_sample);
end
%bno055
if(~isempty(giro_bno)) 
    giro_bno_dps = double(giro_bno)/(2^15)*DPS_MAX_BNO;
else
    giro_bno_dps = zeros(3,n_sample);
end
if(~isempty(acc_bno)) 
    acc_bno_g = double(acc_bno)/(2^13)*G_MAX_BNO;
else
    acc_bno_g = zeros(3,n_sample);
end
if(~isempty(mag_bno)) 
    mag_bno_gaus(1,:) = double(mag_bno(1,:))/(2^14)*MICRO_TESLA_MAX;
    mag_bno_gaus(2,:) = double(mag_bno(2,:))/(2^14)*MICRO_TESLA_MAX;
    mag_bno_gaus(3,:) = double(mag_bno(3,:))/(2^14)*MICRO_TESLA_MAX;
else
    mag_bno_gaus = zeros(3,n_sample);
end

if(isempty(Q)) 
   Q = zeros(4,n_sample); 
end 

% [b,a] = butter(4,0.1);
% filt_giro = filter(b,a,giro_imu_dps(2,1:n_sample));
% figure(10)
% plot(t_imu,[giro_imu_dps(2,1:n_sample); filt_giro]');

%% plots

figure(3);
subplot(311);plot(t_imu,[giro_imu_dps(1,1:n_sample); giro_bno_dps(1,1:n_sample)]'); grid on;
title('Giroscópio');ylabel('X [dps]');legend('Minimu','BNO','Location','northwest','Orientation','horizontal')
subplot(312);plot(t_imu,[giro_imu_dps(2,1:n_sample); giro_bno_dps(2,1:n_sample)]'); ylabel('Y [dps]'); grid on;
subplot(313);plot(t_imu,[giro_imu_dps(3,1:n_sample); giro_bno_dps(3,1:n_sample)]'); ylabel('Z [dps]'); grid on;
xlabel('Tempo [s]')

figure(4);
subplot(311);plot(t_imu,[acc_imu_g(1,1:n_sample); acc_bno_g(1,1:n_sample)]'); grid on;
title('Acelerômetro');ylabel('X [g]');legend('Minimu','BNO','Location','northwest','Orientation','horizontal')
subplot(312);plot(t_imu,[acc_imu_g(2,1:n_sample); acc_bno_g(2,1:n_sample)]'); ylabel('Y [g]'); grid on;
subplot(313);plot(t_imu,[acc_imu_g(3,1:n_sample); acc_bno_g(3,1:n_sample)]'); ylabel('Z [g]'); grid on;

figure(5);
subplot(311);plot(t_imu,[mag_imu_gaus(1,1:n_sample); mag_bno_gaus(1,1:n_sample)]'); grid on; grid on;
title('Magnetômetro');ylabel('X');legend('Minimu','BNO','Location','northwest','Orientation','horizontal')
subplot(312);plot(t_imu,[mag_imu_gaus(2,1:n_sample); mag_bno_gaus(2,1:n_sample)]'); ylabel('Y'); grid on;
subplot(313);plot(t_imu,[mag_imu_gaus(3,1:n_sample); mag_bno_gaus(3,1:n_sample)]'); ylabel('Z'); grid on;
xlabel('Tempo [s]')

figure(6);
plot(t_imu,Q'); grid on;
title('Quaternios'); legend('w','x','y','z');

figure(7);
subplot(121);
plot(mag_imu_gaus(1,:),mag_imu_gaus(2,:),'.');hold all;
plot(mag_imu_gaus(1,:),mag_imu_gaus(3,:),'.');
plot(mag_imu_gaus(3,:),mag_imu_gaus(2,:),'.');
w = linspace(0,2*pi);plot(cos(w),sin(w),'r');
title('Elipsoide Minimu'); legend('XY','XZ','ZY');
hs(2) = subplot(122);
scatter3(hs(2),mag_imu_gaus(1,:),mag_imu_gaus(2,:),mag_imu_gaus(3,:),'*')

figure(8);
subplot(121);
plot(mag_bno_gaus(1,:),mag_bno_gaus(2,:),'.');hold all;
plot(mag_bno_gaus(1,:),mag_bno_gaus(3,:),'.');
plot(mag_bno_gaus(3,:),mag_bno_gaus(2,:),'.');
w = linspace(0,2*pi);plot(cos(w),sin(w),'r');
title('Elipsoide BNO'); legend('XY','XZ','ZY');
hs(2) = subplot(122);
scatter3(hs(2),mag_bno_gaus(1,:),mag_bno_gaus(2,:),mag_bno_gaus(3,:),'*');

