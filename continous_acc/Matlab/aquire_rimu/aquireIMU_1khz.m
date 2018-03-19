clear all; clc; 
close all;
fclose(instrfind);

%% Definição dos parametros iniciais do ensaio
BLUETOOTH = 0;          %se for usar bluetooth sete este define, se for usar 
                        %conversor serial USB configure em 0
time_sample = 10;       %número de segundos a se coletar
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

G_MAX = 4;
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
x_giro = zeros(1,n_amostras); %prealocação para melhora da performance
y_giro = zeros(1,n_amostras);
z_giro = zeros(1,n_amostras);
x_acc = zeros(1,n_amostras); 
y_acc = zeros(1,n_amostras);
z_acc = zeros(1,n_amostras);
x_mag = zeros(1,n_amostras); 
y_mag = zeros(1,n_amostras);
z_mag = zeros(1,n_amostras);

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
    if c_scale < 10
        while (bytes < 12)
                 bytes = s_imu.BytesAvailable;
        end
        x_giro(j) = fread(s_imu,1,'int16');
        y_giro(j) = fread(s_imu,1,'int16');
        z_giro(j) = fread(s_imu,1,'int16');
        x_acc(j) = fread(s_imu,1,'int16');
        y_acc(j) = fread(s_imu,1,'int16');
        z_acc(j) = fread(s_imu,1,'int16');
    else
        while (bytes < 18)
                 bytes = s_imu.BytesAvailable;
        end
        x_giro(j) = fread(s_imu,1,'int16');
        y_giro(j) = fread(s_imu,1,'int16');
        z_giro(j) = fread(s_imu,1,'int16');
        x_acc(j) = fread(s_imu,1,'int16');
        y_acc(j) = fread(s_imu,1,'int16');
        z_acc(j) = fread(s_imu,1,'int16');
        x_mag(j) = fread(s_imu,1,'int16');
        y_mag(j) = fread(s_imu,1,'int16');
        z_mag(j) = fread(s_imu,1,'int16'); 
        c_scale = 0;
    end
end

flushinput(s_imu);
fclose(instrfind);

%% Pós processing
x_giro_dps = double(x_giro)/(2^15)*DPS_MAX;
y_giro_dps = double(y_giro)/(2^15)*DPS_MAX;
z_giro_dps = double(z_giro)/(2^15)*DPS_MAX;

x_acc_g = double(x_acc)/(2^15)*G_MAX;
y_acc_g = double(y_acc)/(2^15)*G_MAX;
z_acc_g = double(z_acc)/(2^15)*G_MAX;

x_mag_gaus = double(x_mag)/(2^15)*MICRO_TESLA_MAX;
y_mag_gaus = double(y_mag)/(2^15)*MICRO_TESLA_MAX;
z_mag_gaus = double(z_mag)/(2^15)*MICRO_TESLA_MAX;

%total_field = sqrt(x_mag_gaus.^2 + 
%% Plots
figure;
plot(t,[x_giro_dps; y_giro_dps; z_giro_dps]'); grid on;
title('Giroscópio');
xlabel('Tempo [s]'); ylabel('dps'); 
legend('x','y','z');

figure;
plot(t,[x_acc_g;y_acc_g;z_acc_g]'); grid on;
title('Acelerômetro');
xlabel('Tempo [s]'); ylabel('g');
legend('x','y','z');

figure;
plot(t_mag,[x_mag_gaus;y_mag_gaus;z_mag_gaus]'); grid on;
title('Magnetômetro');
xlabel('Tempo [s]'); ylabel('Micro Tesla');
legend('x','y','z');
