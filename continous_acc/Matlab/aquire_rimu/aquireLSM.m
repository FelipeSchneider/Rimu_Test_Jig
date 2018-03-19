clear all; clc; 
close all;
%fclose(instrfind);

%% Definição dos parametros iniciais do ensaio
BLUETOOTH = 0;          %se for usar bluetooth sete este define, se for usar 
                        %conversor serial USB configure em 0
time_sample = 10;       %número de segundos a se coletar
fs = 500;              %Frequência de amostragem (depende do micro)
n_amostras = time_sample*fs;

t = 0:1/fs:time_sample; %define o vetor de tempo
t(end) = [];            %elimina a ultima posição do vetor tempo

COM_imu = 7;            %Numero da porta COM
COM_imu_baud = 230400; %Baud rate
COM_header_1 = 83;       %Header de inicio da comunicação
COM_header_2 = 172;      %Header de inicio da comunicação
ACTION = 160;            %ação: ler apenas o LSM

G_MAX = 4;
DPS_MAX = 1000;

%% Inicialização das portas e variáveis
COM_imu_name = sprintf('COM%d',COM_imu);
giro_imu = zeros(3,n_amostras); %prealocação para melhora da performance
acc_imu = zeros(3,n_amostras); 

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

msg = [COM_header_1 COM_header_2 ACTION time_sample_hi time_sample_lo];
fwrite(s_imu,msg,'uint8');
disp('Mensagem inicial enviada')

for j=1:n_amostras
        if(mod(j,fs)==0)                    %print o segundo atual
            fprintf('Segundo %d \r',j/fs)
        end
        while (bytes < 12)
                 bytes = s_imu.BytesAvailable;
        end
        giro_imu(:,j) = fread(s_imu,3,'int16');
        acc_imu(:,j) = fread(s_imu,3,'int16');
end

flushinput(s_imu);
fclose(instrfind);

%% Pós processing
giro_imu_dps = double(giro_imu)/(2^15)*DPS_MAX;
acc_imu_g = double(acc_imu)/(2^15)*G_MAX;

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