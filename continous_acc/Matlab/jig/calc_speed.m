speed_top = diff(real_top_angle)/(t(2)-t(1));
mean(abs(speed_top))

speed_base = diff(real_base_angle)/(t(2)-t(1));
mean(abs(speed_base))