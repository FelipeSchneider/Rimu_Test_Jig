clear all; clc; 
close all;
fclose(instrfind);

%% Definição dos parametros iniciais
BLUETOOTH = 0;          %se for usar bluetooth sete este define, se for usar 
                        %conversor serial USB configure em 0
COM_imu = 16;             %Numero da porta COM
COM_imu_baud = 230400;   %Baud rate
COM_header_1 = 83;       %Header de inicio da comunicação
COM_header_2 = 172;      %Header de inicio da comunicação
ACTION = 176;            %ação: ler o status do BNO

%% Inicialização das portas e variáveis
COM_imu_name = sprintf('COM%d',COM_imu);
if(BLUETOOTH == 0)
    disp('iniciando Conversor Serial USB')
    s_imu = serial(COM_imu_name,'BaudRate',COM_imu_baud,'DataBits',8);
else
    disp('iniciando Bluetooth')
    %s_imu = Bluetooth('RIMU',1);
     H-C-2010-06-01
    disp('Conectando')
end
fopen(s_imu);
flushinput(s_imu);

%%comunicação
msg = [COM_header_1 COM_header_2 ACTION 0 0];
fwrite(s_imu,msg,'uint8');
disp('Mensagem inicial enviada')

bytes = 0;
while (bytes < 7)
                 bytes = s_imu.BytesAvailable;
end

header1 = fread(s_imu,1,'uint8');   %must be 0x53   83
header2 = fread(s_imu,1,'uint8');   %must be 0xCA   202
action = fread(s_imu,1,'uint8');    %must be 0xC1   193
calibration = fread(s_imu,1,'uint8');
auto_test = fread(s_imu,1,'uint8');
sys_status = fread(s_imu,1,'uint8');
clock_status = fread(s_imu,1,'uint8');

flushinput(s_imu);
fclose(instrfind);

%% processamento
global_calibration = bitand(calibration, 192)/64;
gyro_calibration = bitand(calibration, 48)/16;
acc_calibration = bitand(calibration, 12)/4;
mag_calibration = bitand(calibration, 3);

mcu_auto_test = auto_test & 8;
gyro_auto_test = auto_test & 4;
mag_auto_test = auto_test & 2;
acc_auto_test = auto_test & 1;


%% Displays
disp('CALIBRAÇÃO:')
switch global_calibration
    case 3
        disp('O sistema está globalmente totalmente calibrado')
    case 2
        disp('O sistema está globalmente bem calibrado')
    case 1
        disp('O sistema está globalmente pouco calibrado')
    case 0
        disp('O sistema está globalmente descalibrado')
end

switch gyro_calibration
    case 3
        disp('O giroscópio está totalmente calibrado')
    case 2
        disp('O giroscópio está bem calibrado')
    case 1
        disp('O giroscópio está pouco calibrado')
    case 0
        disp('O giroscópio está descalibrado')
end

switch acc_calibration
    case 3
        disp('O acelerômetro está totalmente calibrado')
    case 2
        disp('O acelerômetro está bem calibrado')
    case 1
        disp('O acelerômetro está pouco calibrado')
    case 0
        disp('O acelerômetro está descalibrado')
end

switch mag_calibration
    case 3
        disp('O magnetômetro está totalmente calibrado')
    case 2
        disp('O magnetômetro está bem calibrado')
    case 1
        disp('O magnetômetro está pouco calibrado')
    case 0
        disp('O magnetômetro está descalibrado')
end


disp(' ');
disp('AUTO TESTE:');
if(mcu_auto_test)
    disp('O microcontrolador do BNO foi aprovado no auto teste')
else
    disp('O microcontrolador do BNO falhou no auto teste')
end

if(gyro_auto_test)
    disp('O giroscópio do BNO foi aprovado no auto teste')
else
    disp('O giroscópio do BNO falhou no auto teste')
end

if(mag_auto_test)
    disp('O magnetômetro do BNO foi aprovado no auto teste')
else
    disp('O magnetômetro do BNO falhou no auto teste')
end

if(acc_auto_test)
    disp('O acelerômetro do BNO foi aprovado no auto teste')
else
    disp('O acelerômetro do BNO falhou no auto teste')
end

disp(' ');
disp('SYSTEM STATUS:')
switch sys_status
    case 0
        disp('Sistema em idle')
    case 1
        disp('Sistema em ERRO')
    case 2
        disp('Inicializando periféricos')
    case 3
        disp('Sistema em inicialização')
    case 4
        disp('Executando auto teste')
    case 5
        disp('Rodando SEM algoritmo de fusão')
    case 6
        disp('Rodando COM algoritmo de fusão')
end

disp(' ');
disp('CLOCK STATUS');
if(clock_status)
    disp('Não altere a fonte de clock')
else
    disp('Fonte de clock livre para alteração')
end