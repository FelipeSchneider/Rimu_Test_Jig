function [com_vector, b_angles, t_angles] = createComVector( base_angle, base_speed, top_angle, top_speed , jig_const)
%[com_vector, b_angles, t_angles] = createComVector( base_angle, base_speed, top_angle, top_speed , d_theta)
%Create the command vector that will be send to the microcontroler afterward
%   base_angle      set of angles (in degrees) that the base motor will move. 
%                   The direction will be given by the signal of speed. The
%                   base motor has a limit for turning in the same
%                   direction, this limit is programmed in firmware and it
%                   is now set to 720 degrees.
%   base_speed      set of cruse speeds (positive-> CCW, negative -> CW)
%                   that the motor will apply if it is possible to reach the
%                   given cruse speed
%   top_angle       similar to the base_angle for the top motor. The top
%                   does not have a turning limitation
%   top_speed       similar to the base speed for the top motor
%   jig_const       The constants structure related to the jig
%   com_vector      command vector that will be send to the microcontroller
%   b_angles        The final set of angles described by the base motor
%                   real angles - (double)
%   t_angles        The final set of angles described by the top motor
%                   real angles - (double)
%
%
%  Communication protocol
%  First byte -> UART_HEADER_1
%  Second byte -> UART_HEADER_COM for sending the squence of steps and UART_HEADER_START to start executing the squence of steps
%  From the third byte, we start the action section: two bytes for the cruse speed of base (negative means one direction, positive another)
%  another two bytes for the number of steps in this direction (base). The next for bytes are the cruse speed of top and number of steps for top motor.
%  So the protocol will be like this:
%  HEADER1, HEADER2, S_BASE_HIGH, S_BASE_LOW, N_BASE_HIGH, N_BASE_LOW, S_TOP_HIGH, S_TOP_LOW, N_TOP_HIGH, N_TOP_LOW, ...N*8 Bytes ..., 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00
% 							 int16_t				  uint16_t                 int16_t               uint16_t                      END_MSG - MATLAB SENDS FIRST THE LOW BYTE
d_theta = jig_const.d_theta;
max_size = jig_const.limit_com_size;
L_ba = length(base_angle);
L_bs = length(base_speed);
L_ta = length(top_angle);
L_ts = length(top_speed);

%check if there are some speed above the maximum speed
if(max(base_speed)>jig_const.max_w)
     warning('Base speed higher than the maximum allowed');
     base_speed(base_speed>jig_const.max_w) = 720;  % limiting the speed
end

if(max(top_speed)>jig_const.max_w)
     warning('Top speed higher than the maximum allowed');
     top_speed(top_speed>jig_const.max_w) = 720;  % limiting the speed
end


%check to verify is the number of commands is bigger than what is 
if(L_ba> max_size) %249 is the size that is defined in the jig microcontroler
    warning('The number of comands is bigger than the microcontroller buffer size');
    warning('cutting the number of base angles commands');
    base_angle(max_size:end) = [];
end
    
if(L_bs> max_size)
    warning('The number of comands is bigger than the microcontroller buffer size');
    warning('cutting the number of base speed commands');
    base_speed(max_size:end) = [];
end  

if(L_ta> max_size)
    warning('The number of comands is bigger than the microcontroller buffer size');
    warning('cutting the number of top angles commands');
    top_angle(max_size:end) = [];
end
    
if(L_ts> max_size)
    warning('The number of comands is bigger than the microcontroller buffer size');
    warning('cutting the number of top speed commands');
    top_speed(max_size:end) = [];
end  

if(L_ba>L_bs)
    warning('Length of base_angle vector is not equal to base speed vector');
    warning('The lasts angles will be discarted');
    base_angle(L_bs+1:end) = [];
elseif(L_ba<L_bs)
    warning('Length of base_angle vector is not equal to base speed vector');
    warning('The lasts speeds will be discarted');
    base_speed(L_ba+1:end) = [];
end

if(L_ta>L_ts)
    warning('Length of top_angle vector is not equal to top speed vector');
    warning('The lasts angles will be discarted');
    top_angle(L_ts+1:end) = [];
elseif(L_ta<L_ts)
    warning('Length of top_angle vector is not equal to top speed vector');
    warning('The lasts speeds will be discarted');
    top_speed(L_ta+1:end) = [];
end

n_steps_b = round(base_angle./d_theta); %calculating the number of steps of each command for the base motor
if(max(n_steps_b) > 2^16-1)
    warning('applying uint16 limit to the number of steps of base motor');
    n_steps_b(n_steps_b>2^16-1) = 2^16-1;
end
b_angles = zeros(1,L_ba+1);
for i=1:L_ba                            %calculating the real angle that the motor will describe
    if(base_speed(i) > 0)
        b_angles(i+1) = b_angles(i)+ n_steps_b(i)*d_theta;
    elseif(base_speed(i)<0)
        b_angles(i+1) = b_angles(i) - n_steps_b(i)*d_theta;
    else
        b_angles(i+1) = b_angles(i);
        n_steps_b(i) = base_angle(i); %when the speed is smaller than the minimum speed, then we just have to make a delay
    end
end

n_steps_t = round(top_angle./d_theta);
if(max(n_steps_t) > 2^16-1)
    warning('applying uint16 limit to the number of steps of top motor');
    n_steps_t(n_steps_t>2^16-1) = 2^16-1;
end
t_angles = zeros(1,L_ba+1);
for i=1:L_ta
    if(top_speed(i) > 0)
        t_angles(i+1) = t_angles(i)+ n_steps_t(i)*d_theta;
    elseif(top_speed(i)<0)
        t_angles(i+1) = t_angles(i) - n_steps_t(i)*d_theta;
    else
        t_angles(i+1) = t_angles(i);
        n_steps_t(i) = top_angle(i); %when the speed is smaller than the minimum speed, then we just have to make a delay
    end
end

com_vector = 0;
if(L_ta<L_ba)              %if the number of comands for the base is bigger than the number of comands to the top
    top_speed(L_ba) = 0;
    n_steps_t(L_ba) = 0;
elseif(L_ta>L_ba)          %if the number of comands to the top is bigger
    base_speed(L_ta) = 0;
    n_steps_b(L_ta) = 0;
end

for i=1:L_ta
    com_vector = [com_vector base_speed(i) n_steps_b(i) top_speed(i) n_steps_t(i)];
end

com_vector(1) = [];
com_vector = [com_vector -32768 0 -32768 0]; %end of msg
end

