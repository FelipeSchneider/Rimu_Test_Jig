function [ e_sensor ] = angleAlignment( e_sensor, e_jig, pol_yaw, pol_roll, pol_pitch, axis)

    if(strcmp(axis,'yaw'))
        a = 1;
    elseif(strcmp(axis,'roll'))
        a = 2;
    elseif(strcmp(axis,'pitch'))
        a = 3;
    else
        error('Axis not valid');
    end

    yaw_comp = (e_jig(:,a).^4*pol_yaw.p1 + e_jig(:,a).^3*pol_yaw.p2 + e_jig(:,a).^2*pol_yaw.p3 + e_jig(:,a)*pol_yaw.p4 + pol_yaw.p5);
    roll_comp = (e_jig(:,a).^4*pol_roll.p1 + e_jig(:,a).^3*pol_roll.p2 + e_jig(:,a).^2*pol_roll.p3 + e_jig(:,a)*pol_roll.p4 + pol_roll.p5);
    pitch_comp = (e_jig(:,a).^4*pol_pitch.p1 + e_jig(:,a).^3*pol_pitch.p2 + e_jig(:,a).^2*pol_pitch.p3 + e_jig(:,a)*pol_pitch.p4 + pol_pitch.p5);

    e_sensor(:,1) = e_sensor(:,1)+yaw_comp;  
    e_sensor(:,2) = e_sensor(:,2)+roll_comp;  
    e_sensor(:,3) = e_sensor(:,3)+pitch_comp;

end

