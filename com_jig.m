clear all; clc; 
close all;
%fclose(instrfind);
%% angles and speed configurations - user configurable up to 
% top_angle = [90 90 90 90 90 90];
% top_speed = [15 60 120 -120 -60 -20];
% 
% base_angle = [90 90 90 90 90 90];
% base_speed = [30 120 180 -180 -120 -15];

top_angle = [90 90 90 90 90 90 120 150 800 20 10 10 30];
top_speed = [35 60 120 -120 -60 -20 120 300 -720 12 -30 50 -30];

base_angle = [150 90 90 90 90 90];
base_speed = [350 120 180 -180 -120 -15];

%% constants
gig_const = struct('microstep', 16, 'step_per_rev', 200,... 
            'd_theta', 360/(16*200),'min_w',10,'max_w',720,...
            'max_base_angle',720,'acc_mod',720,'breaking_mod',720,...
            'positive_speed','CCW','negative_speed','CW',...
            'limit_com_size',249);
        
%% calculations
disp('Calculating the commands');
[com_vector, b_angles, t_angles] = createComVector(base_angle, base_speed, top_angle, top_speed ,gig_const);
com_vector2 = typecast(int32(com_vector), 'uint16');
com_vector2 = com_vector2(1:2:end); %taking only the odd positions

[t, real_base_angle, real_top_angle ] = reconstructCinematic( com_vector, gig_const);
figure(1);
subplot(211);plot(t,real_base_angle); title('Jig angles'); ylabel('Base angles (°)');
subplot(212);plot(t,real_top_angle);xlabel('Time [s]'); ylabel('Top angles (°)');

figure(2);
subplot(211); plot(repelem(b_angles,5)); title('Ilustrative jig angles');
ylabel('Base angles (°)'); 
subplot(212);
plot(repelem(t_angles,5)); ylabel('Top angles (°)'); xlabel('commands sequence');



%% Start of communication 
header_start_gig = 'St';
header_command_gig = 'Sc';
COM_gig_num = 11;   COM_gig_name = sprintf('COM%d',COM_gig_num);
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
pause(4);

fclose(instrfind);

