% This script send instructions to the jig, and only to the jig.
% Those instructions can be changed by the user changing the 4 next variables:
% top_angle, top_speed, base_angle and base_speed.
% the speed is given in degrees per second, while the angle is in degrees.
% if the "speed" of any given motor is equaled to zero, the "angle" will
% indicate the time in milliseconds that the given motor will stall.
% if one motor finishes the movement before the other, it will wait for the other finishes
% before a new command starts.

clear all; clc; 
close all;
%fclose(instrfind);
%% angles and speed configurations - user configurable up to 249 commands for each motor
% top_angle = [90 90 90 90 90 90];
% top_speed = [15 60 120 -120 -60 -20];
% 
% base_angle = [90 90 90 90 90 90];
% base_speed = [30 120 180 -180 -120 -15];

% top_angle = [90 90 90 90 90 90 120 150 3000 800 20 10 10 30];
% top_speed = [35 60 120 -120 -60 -20 120 300 0 -720 12 -30 50 -30];
% 
% base_angle = [150 90 90 90 90 90];
% base_speed = [350 120 180 -180 -120 -15];

% top_angle = [360 270 450 360 90 90 400 180 720 720 60 360 720 540 300];
% top_speed = [300 -300 720 -400 100 40 100 -300 720 -720 100 -720 300 -250 50];
% 
% base_angle = [360 270 450 360 90 90 400 180 500 500 60 360 720 540 300];
% base_speed = [300 -300 720 -400 100 -40 100 -300 -720 720 -100 -720 300 -250 50];

% top_angle = [7000 5000 360 360 5000 5000 360 360];
% top_speed = [0 0 60 60 0 0 -60 -60];
% 
% base_angle = [7000 360 5000 360 5000 360 5000 360];
% base_speed = [0 60 0 -60 0 -60 0 60];

%random test
% base_angle = [5000 30  60  10  1000 120  30    45  80   1000   150 60   75  300  15  80  300  120 60 1000  60 120 200 45  150  15  30    1000  85  145 360  60 60 60  60 1000 120 75   2000];
% base_speed = [0     200 720 -45 0    -150 45    -80 -300 0      600 -450 360 90  -99 145 -120 720 64 0     89 254 -99 130 -89 -78 -60   0    320 187 -180 60 30 180   20 0    109  -500 0];
% top_angle = [5000 45   99  120  1000 320  15    45  240  1000   75  480  65  200 99  46  135  220 90 1000  20 220 170 205  20  425  30   1000 145 115 260  60 60  450  60 1000 120 175  2000];
% top_speed = [0     200 720 -45  0    -150 45    -90 -300 0      600 -450 360 90  -99 145 -120 720 64 0     89 254 -99 330 -89 -320 -70   0    320 187 -180 60 -30 180  55 0    109  -500 0];

%fast speed
base_angle = [5000 90  120 50   150  30  180 480  30  60   360 450   90   150 145 175 25  35   90  78  14   423  165 245 720  250 25  25  25  400  360 142 15  25  35  165  85];
base_speed = [0    200 300 -400 -500 115 720 -180 100 -245 720 -250 -130 -650 600 165 278 -145 -75 -98 -125 145  720 500 -450 524 250 152 365 -654 250 350 152 365 184 -500 410];
top_angle = [5000 85  165  35  25  15  142 360 4000 25  25  25  2500 720  245 165  120  14  78  90  35   25  1705 145 150  90   80   360  60  30  180  180 30  150   50 120  90];
top_speed = [0    410 -500 184 365 152 350 250 -654 365 152 250 524  -450 500 720 -145 -125 -98 -75 -145 278 265  600 -650 -130 -100 720 -245 100 -180 720 115 -500 -400 300 200];


COM_gig_num = 11;

%% constants
gig_const = struct('microstep', 16, 'step_per_rev', 200,... 
            'd_theta', 360/(16*200),'min_w',10,'max_w',720,...
            'max_base_angle',720,'acc_mod',360,'breaking_mod',360,...
            'positive_speed','CCW','negative_speed','CW',...
            'limit_com_size',249);
        
%% calculations
disp('Calculating the commands');
[com_vector, b_angles, t_angles] = createComVector(base_angle, base_speed, top_angle, top_speed ,gig_const);
com_vector2 = typecast(int32(com_vector), 'uint16');
com_vector2 = com_vector2(1:2:end); %taking only the odd positions

[t, real_base_angle, real_top_angle, com_data ] = reconstructCinematic( com_vector, gig_const);
figure(1);
subplot(211);plot(t,real_base_angle); hold on;
plot(com_data(1,:),com_data(2,:),'*');
title('Jig angles'); ylabel('Base angles (°)'); 
subplot(212);plot(t,real_top_angle); hold on;
plot(com_data(1,:),com_data(3,:),'*');
xlabel('Time [s]'); ylabel('Top angles (°)');

figure(2);
subplot(211); plot(repelem(b_angles,5)); title('Ilustrative jig angles');
ylabel('Base angles (°)'); 
subplot(212);
plot(repelem(t_angles,5)); ylabel('Top angles (°)'); xlabel('commands sequence');



%% Start of communication 
%  the board will answer with an 'S' the start of a set of commands, 
%  with an 'N' every new command and with an 'E' the end of the set of commands
header_start_gig = 'St';
header_command_gig = 'Sc';
COM_gig_name = sprintf('COM%d',COM_gig_num);
COM_gig_baud = 57600;
s_gig = serial(COM_gig_name,'BaudRate',COM_gig_baud,'DataBits',8);
s_gig.InputBufferSize = 2^16;
disp('Opening the COM port')
fopen(s_gig);
disp('Waiting for arduino reset')
pause(3);       %waiting arduino to reset
flushinput(s_gig);

disp('Sending the position commands')
fwrite(s_gig,uint8(header_command_gig),'uint8');
fwrite(s_gig,com_vector2,'uint16');
disp('Sendind the start command')
fwrite(s_gig,uint8(header_start_gig),'uint8');

rec = 0;
bytes = 0;
t_rec = 0;
tic
while(rec ~= 'E')
    while (bytes < 1)
        bytes = s_gig.BytesAvailable;
    end
    rec = fread(s_gig,1,'uint8');  %blower positivo
    t_rec = [t_rec toc];
    if(rec == 'S')
        disp('Start of the set of commands, time:')
        disp(t_rec(end))
    elseif(rec == 'N')
        disp('New command, time:')
        disp(t_rec(end))
    elseif(rec == 'E')
        disp('End of the set of commands, time:')
        disp(t_rec(end))
    else
        disp('Byte received not recognize, time:')
        disp(t_rec(end))
    end
end

fclose(s_gig);

