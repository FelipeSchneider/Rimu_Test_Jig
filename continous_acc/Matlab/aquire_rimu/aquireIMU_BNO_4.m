clear all; clc; 
close all;
fclose(instrfind);

%% Definição dos parametros iniciais do ensaio
BLUETOOTH = 1;          %se for usar bluetooth sete este define, se for usar 

WAIT_SYNC = 0;          %wait for the technaid synchronization or not
time_sample = 30;       %número de segundos a se coletar
fs = 100;               %Frequência de amostragem (depende do micro)
n_amostras = time_sample*fs;

t = 0:1/fs:time_sample; %define o vetor de tempo
t(end) = [];            %elimina a ultima posição do vetor tempo

PLOT_CIRC_MAG = 0;

COM_imu = 16;            %Numero da porta COM
COM_imu_baud = 230400;   %Baud rate
COM_header_1 = 83;       %Header de inicio da comunicação
COM_header_2 = 172;      %Header de inicio da comunicação

if(WAIT_SYNC == 0)
    ACTION = 165;       %ação: ler apenas o LSM+LIS
else
   ACTION = 169;        %ação: ler apenas o LSM+LIS esperando o sincronismo
end

G_MAX_IMU = 8;
G_MAX_BNO = 8;
DPS_MAX_IMU = 1000;
DPS_MAX_BNO = 2000;
GAUSS_MAX_IMU = 4;
MICRO_TESLA_XY_MAX = 1300;
MICRO_TESLA_Z_MAX = 2500;
%bno states that for x and y the full range is 1300 uT and for z is 2500 uT
% 1 uT = 0.01G
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
mag_imu = zeros(3,n_amostras); 

giro_bno = zeros(3,n_amostras); %prealocação para melhora da performance
acc_bno = zeros(3,n_amostras); 
mag_bno = zeros(3,n_amostras);
Q = zeros(4,n_amostras);
%pre-processing minimu
giro_imu_dps = zeros(3,n_amostras);
acc_imu_g = zeros(3,n_amostras);
%pre-processing bno055 
giro_bno_dps = zeros(3,n_amostras);
acc_bno_g = zeros(3,n_amostras);

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
%% plot inicial
screensize = get(groot,'Screensize');
figure(1);
set(gcf, 'Position', [0 0 screensize(3)/2 screensize(4)]); 
subplot(311);
fig_giro_x = plot(t,[giro_imu_dps(1,:); giro_bno_dps(1,:)]'); 
grid on; title('Giroscópio');ylabel('X [dps]');
legend('Minimu','BNO','Location','northwest','Orientation','horizontal')
subplot(312);
fig_giro_y = plot(t,[giro_imu_dps(2,:); giro_bno_dps(2,:)]'); ylabel('Y [dps]'); grid on;
subplot(313);
fig_giro_z = plot(t,[giro_imu_dps(3,:); giro_bno_dps(3,:)]'); ylabel('Z [dps]'); grid on;

figure(2);
set(gcf, 'Position', [screensize(3)/2 0 screensize(3)/2 screensize(4)]); 
subplot(311);fig_acc_x = plot(t,[acc_imu_g(1,:); acc_bno_g(1,:)]'); 
grid on; title('Acelerômetro');ylabel('X [g]');
legend('Minimu','BNO','Location','northwest','Orientation','horizontal')
subplot(312);
fig_acc_y = plot(t,[acc_imu_g(2,:); acc_bno_g(2,:)]'); ylabel('Y [g]'); grid on;
subplot(313);fig_acc_z = plot(t,[acc_imu_g(3,:); acc_bno_g(3,:)]'); ylabel('Z [g]'); grid on;
xlabel('Tempo [s]')

%% Coleta de valores
cont_exit = 0; bytes = 0;
press = 0;
disp('iniciando a coleta')   

time_sample_hi = floor(time_sample/255);
time_sample_lo = time_sample-time_sample_hi*255;

msg = [COM_header_1 COM_header_2 ACTION time_sample_hi time_sample_lo];
fwrite(s_imu,msg,'uint8');
disp('Mensagem inicial enviada')

for j=1:n_amostras
        if(mod(j,fs)==0)                    %print o segundo atual
            fprintf('Segundo %d \r',j/fs)
        end
        while (bytes < 44)
                 bytes = s_imu.BytesAvailable;
        end
        giro_imu(:,j) = fread(s_imu,3,'int16');
        acc_imu(:,j) = fread(s_imu,3,'int16');
        mag_imu(:,j) = fread(s_imu,3,'int16');
        
        acc_bno(:,j) = fread(s_imu,3,'int16');
        mag_bno(:,j) = fread(s_imu,3,'int16');
        giro_bno(:,j) = fread(s_imu,3,'int16');
        
        Q(:,j) = fread(s_imu,4,'int16');
        
        
        giro_imu_dps(:,j) = double(giro_imu(:,j))/(2^15)*DPS_MAX_IMU;
        giro_bno_dps(:,j) = double(giro_bno(:,j))/(2^15)*DPS_MAX_BNO;
        
        acc_imu_g(:,j) = double(acc_imu(:,j))/(2^15)*G_MAX_IMU;
        acc_bno_g(:,j) = double(acc_bno(:,j))/(2^13)*G_MAX_BNO;
        if(mod(j,fs/1)==0) 
            %fig_giro_x(1).YData(j) = giro_imu_dps(1,j);
            set(fig_giro_x(1),'Ydata',giro_imu_dps(1,:)');
            set(fig_giro_x(2),'Ydata',giro_bno_dps(1,:)');      
            set(fig_giro_y(1),'Ydata',giro_imu_dps(2,:)');
            set(fig_giro_y(2),'Ydata',giro_bno_dps(2,:)');    
            set(fig_giro_z(1),'Ydata',giro_imu_dps(3,:)');
            set(fig_giro_z(2),'Ydata',giro_bno_dps(3,:)');

            set(fig_acc_x(1),'Ydata',acc_imu_g(1,:)');
            set(fig_acc_x(2),'Ydata',acc_bno_g(1,:)'); 
            set(fig_acc_y(1),'Ydata',acc_imu_g(2,:)');
            set(fig_acc_y(2),'Ydata',acc_bno_g(2,:)');      
            set(fig_acc_z(1),'Ydata',acc_imu_g(3,:)');
            set(fig_acc_z(2),'Ydata',acc_bno_g(3,:)');
            %set(fig_giro_z(1),'Xdata',t);
            drawnow
        end
end

flushinput(s_imu);
fclose(instrfind);

%% Pós processing
%minimu
%giro_imu_dps = double(giro_imu)/(2^15)*DPS_MAX_IMU;
%acc_imu_g = double(acc_imu)/(2^15)*G_MAX_IMU;
mag_imu_gaus = double(mag_imu)/(2^15)*GAUSS_MAX_IMU;

%bno055
%giro_bno_dps = double(giro_bno)/(2^15)*DPS_MAX_BNO;
%acc_bno_g = double(acc_bno)/(2^13)*G_MAX_BNO;
mag_bno_gaus(1:2,:) = double(mag_bno(1:2,:))/(2^13)*MICRO_TESLA_XY_MAX/100; %Divide by 100 to convert into gauss
mag_bno_gaus(3,:) = double(mag_bno(3,:))/(2^15)*MICRO_TESLA_Z_MAX/100;

%total_field = sqrt(x_mag_gaus.^2 + 
%% Plots
% figure(1);
% subplot(311);plot(t,[giro_imu_dps(1,:); giro_bno_dps(1,:)]'); grid on;
% title('Giroscópio');ylabel('X [dps]');legend('Minimu','BNO','Location','northwest','Orientation','horizontal')
% subplot(312);plot(t,[giro_imu_dps(2,:); giro_bno_dps(2,:)]'); ylabel('Y [dps]'); grid on;
% subplot(313);plot(t,[giro_imu_dps(3,:); giro_bno_dps(3,:)]'); ylabel('Z [dps]'); grid on;
% xlabel('Tempo [s]')

% figure(2);
% subplot(311);plot(t,[acc_imu_g(1,:); acc_bno_g(1,:)]'); grid on;
% title('Acelerômetro');ylabel('X [g]');legend('Minimu','BNO','Location','northwest','Orientation','horizontal')
% subplot(312);plot(t,[acc_imu_g(2,:); acc_bno_g(2,:)]'); ylabel('Y [g]'); grid on;
% subplot(313);plot(t,[acc_imu_g(3,:); acc_bno_g(3,:)]'); ylabel('Z [g]'); grid on;
% xlabel('Tempo [s]')

figure(3);
subplot(311);plot(t,[mag_imu_gaus(1,:); mag_bno_gaus(1,:)]'); grid on; grid on;
title('Magnetômetro');ylabel('X');legend('Minimu','BNO','Location','northwest','Orientation','horizontal')
subplot(312);plot(t,[mag_imu_gaus(2,:); mag_bno_gaus(2,:)]'); ylabel('Y'); grid on;
subplot(313);plot(t,[mag_imu_gaus(3,:); mag_bno_gaus(3,:)]'); ylabel('Z'); grid on;
xlabel('Tempo [s]')

figure(4);
plot(t,Q'); grid on;
title('Quaternios'); legend('w','x','y','z');

if(PLOT_CIRC_MAG == 1)
    figure(5);
    subplot(121);
    plot(mag_imu_gaus(1,:),mag_imu_gaus(2,:),'.');hold all;
    plot(mag_imu_gaus(1,:),mag_imu_gaus(3,:),'.');
    plot(mag_imu_gaus(3,:),mag_imu_gaus(2,:),'.');
    w = linspace(0,2*pi);plot(cos(w),sin(w),'r');
    title('Elipsoide Minimu'); legend('XY','XZ','ZY');
    hs(2) = subplot(122);
    scatter3(hs(2),mag_imu_gaus(1,:),mag_imu_gaus(2,:),mag_imu_gaus(3,:),'*')

    figure(6);
    subplot(121);
    plot(mag_bno_gaus(1,:),mag_bno_gaus(2,:),'.');hold all;
    plot(mag_bno_gaus(1,:),mag_bno_gaus(3,:),'.');
    plot(mag_bno_gaus(3,:),mag_bno_gaus(2,:),'.');
    w = linspace(0,2*pi);plot(cos(w),sin(w),'r');
    title('Elipsoide BNO'); legend('XY','XZ','ZY');
    hs(2) = subplot(122);
    scatter3(hs(2),mag_bno_gaus(1,:),mag_bno_gaus(2,:),mag_bno_gaus(3,:),'*')
end

clear w hs;