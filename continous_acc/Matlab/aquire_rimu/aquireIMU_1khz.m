clear all; clc; 
close all;
if(~isempty(instrfind))
    disp('Closing all COM ports')
    fclose(instrfind);
end

%% Definição dos parametros iniciais do ensaio
BLUETOOTH = 1;          %se for usar bluetooth sete este define, se for usar 
                        %conversor serial USB configure em 0
time_sample = 30;       %número de segundos a se coletar
fs = 1000;              %Frequência de amostragem (depende do micro)
n_amostras = time_sample*fs;

t = 0:1/fs:time_sample; %define o vetor de tempo
t_mag = 0:1/100:time_sample; %define o vetor de tempo
t(end) = [];            %elimina a ultima posição do vetor tempo
t_mag(end) = [];            %elimina a ultima posição do vetor tempo

COM_imu = 7;            %Numero da porta COM
COM_imu_baud = 230400; %Baud rate
COM_header_1 = 83;       %Header de inicio da comunicação
COM_header_2 = 172;      %Header de inicio da comunicação
ACTION = 167;            %ação: ler apenas o LSM+LIS

G_MAX = 8;
DPS_MAX = 1000;
GAUSS_MAX = 4;
MICRO_TESLA_MAX = 40;

%% dados magnéticos de vitória
%https://www.ngdc.noaa.gov/geomag-web/#igrfwmm
% Model Used:	WMM2015	More information
% Latitude:	20° 19' 59" S
% Longitude:	40° 24' 50" W
% Elevation:	0.0 km Mean Sea Level

% Date 2017-08-24	
% ( + E  | - W ) Declination -23° 44' 36"	
% ( + D  | - U ) Inclination -39° 55' 15"	
% Horizontal Intensity 18136.9 nT	
% (+ N  | - S) North Comp 16601.8 nT	
% (+ E  | - W) East Comp -7302.7 nT	
% (+ D  | - U) Vertical Comp -15176.0 nT	
% Total Field 23648.6 nT

%% Inicialização das portas e variáveis
COM_imu_name = sprintf('COM%d',COM_imu);
giro_imu = zeros(3,n_amostras); %prealocação para melhora da performance
acc_imu = zeros(3,n_amostras); 
mag_imu = zeros(3,n_amostras/10); 

if(BLUETOOTH == 0)
    disp('iniciando Conversor Serial USB')
    s_imu = serial(COM_imu_name,'BaudRate',COM_imu_baud,'DataBits',8);
else
    disp('iniciando Bluetooth')
    s_imu = Bluetooth('RIMU',1);
    disp('Conectando')
end
s_imu.InputBufferSize = n_amostras*20;
fopen(s_imu);
flushinput(s_imu);
pause(1);
%% Coleta de valores
cont_exit = 0; bytes = 0;
press = 0;
disp('iniciando a coleta')   

time_sample_hi = floor(time_sample/255);
time_sample_lo = time_sample-time_sample_hi*255;

fwrite(s_imu,uint8(COM_header_1),'uint8');
pause(0.1);
fwrite(s_imu,uint8(COM_header_2),'uint8');
pause(0.1);
fwrite(s_imu,uint8(ACTION),'uint8');
pause(0.1);
fwrite(s_imu,uint8(time_sample_hi),'uint8');
pause(0.1);
fwrite(s_imu,uint8(time_sample_lo),'uint8');
disp('Mensagem inicial enviada')
c_scale = 0;
for j=1:n_amostras
    c_scale = c_scale + 1;
    if(mod(j,fs)==0)                    %print o segundo atual
        fprintf('Segundo %d \r',j/fs)
    end
     if c_scale < 10                    % envio sem os dados do magnetometro
        bytes = s_imu.BytesAvailable;
        while (bytes < 12)
                 bytes = s_imu.BytesAvailable;
        end
        giro_imu(:,j) = fread(s_imu,3,'int16');
        acc_imu(:,j) = fread(s_imu,3,'int16');
     else                               % envio com os dados do magnetometro
        bytes = s_imu.BytesAvailable;
        while (bytes < 18)
                 bytes = s_imu.BytesAvailable;
        end
        giro_imu(:,j) = fread(s_imu,3,'int16');
        acc_imu(:,j) = fread(s_imu,3,'int16');
        mag_imu(:,round(j/10)) = fread(s_imu,3,'int16'); 
        c_scale = 0;
    end
end

flushinput(s_imu);
fclose(instrfind);

%% Pós processing
giro_imu_dps = double(giro_imu)/(2^15)*DPS_MAX;
acc_imu_g = double(acc_imu)/(2^15)*G_MAX;
mag_imu_gaus = double(mag_imu)/(2^15)*MICRO_TESLA_MAX;

%total_field = sqrt(x_mag_gaus.^2 + 
%% Plots
figure;
subplot(311);plot(t,giro_imu_dps(1,:)); grid on;
title('Giroscópio');ylabel('X [dps]');legend('Minimu','Location','northwest','Orientation','horizontal')
subplot(312);plot(t,giro_imu_dps(2,:)); ylabel('Y [dps]'); grid on;
subplot(313);plot(t,giro_imu_dps(3,:)); ylabel('Z [dps]'); grid on;
xlabel('Tempo [s]')


figure;
subplot(311);plot(t,acc_imu_g(1,:)'); grid on;
title('Acelerômetro');ylabel('X [g]');legend('Minimu','Location','northwest','Orientation','horizontal')
subplot(312);plot(t,acc_imu_g(2,:)'); ylabel('Y [g]'); grid on;
subplot(313);plot(t,acc_imu_g(3,:)'); ylabel('Z [g]'); grid on;
xlabel('Tempo [s]')

t2 = t(1:10:end);
figure;
subplot(311);plot(t2,mag_imu_gaus(1,:)'); grid on; grid on;
title('Magnetômetro');ylabel('X');legend('Minimu','Location','northwest','Orientation','horizontal')
subplot(312);plot(t2,mag_imu_gaus(2,:)'); ylabel('Y'); grid on;
subplot(313);plot(t2,mag_imu_gaus(3,:)'); ylabel('Z'); grid on;
xlabel('Tempo [s]')
