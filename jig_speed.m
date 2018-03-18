close all; clear all;
DEFAULT_MS = 16;
FS_REVOLUTION = 200;
ideal_period = 0;
F_CPU = 16E6;
for speed = 2:720
    ideal_period(speed) = ((1E6*360)/(speed*DEFAULT_MS*FS_REVOLUTION)); %this is in microseconds
end
ideal_cycles = (F_CPU / 2000000) .* ideal_period;     %divide by 2 because the actual interruption just count at rising clock edge

real_cycles = ideal_cycles;
for i=1:15  %up to 15 degrees per second we must prescaler the timer by 8;
  real_cycles(i) = ideal_cycles(i)/8;
end
real_cycles = uint16(round(real_cycles)); %we will keep only the int part -> TRANSFER THIS TO THE PROGRAM

%calculating the real speed that will be apply to the motors
real_period = double(real_cycles)/(F_CPU / 2000000);
for i=1:15  %up to 15 degrees per second we must prescaler the timer by 8;
  real_period(i) = real_period(i)*8;
end

real_speed = ((1E6*360)./(double(real_period)*DEFAULT_MS*FS_REVOLUTION)); %this is in microseconds
real_speed(1) = 0;
% plot(cycles); hold on; plot(cycles_scaler);
figure(1);
plot(ideal_period); hold on; plot(real_period); 
title('Timer period');
legend('Ideal period','Possible period');xlabel('speed'); ylabel('Period (clock cycles)');
figure(2);
plot(real_speed); title('Real possible speeds'); 
xlabel('ideal ang. speed (degrees per second)');
ylabel('real ang. speed (degrees per second)');
