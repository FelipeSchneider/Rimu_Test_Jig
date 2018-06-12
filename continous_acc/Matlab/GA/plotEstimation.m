function [ HP ] = plotEstimation(t, q_all, q_best, q_jig, f_plot, nIteration, HP )
%plotEstimation Plot the estimation of all particles, including the best estimation
%and the jig angle
%   t     - Time vector
%   q_all - %the roll distribution is:
            %first position the time stamp
            %second position define the quaternion
            %third position define the particle
%   q_best - best quaternion estimation
%   q_jig  - jig quaternion (reference)
%   f_plot - plot the graphs or nor
%   nIteration - number of the iteration to that will apear in the title
%   HP - Handle plot of the graph

q_all(:,:,1); %calculate the variance with this
if f_plot == 1
    [e_jig(:,1),e_jig(:,2),e_jig(:,3)] = quat2angle(q_jig,'ZXY');
    [e_best(:,1),e_best(:,2),e_best(:,3)] = quat2angle(q_best,'ZXY');
    
    e_diff(:,1) = angdiff(abs(e_jig(:,1)),abs(e_best(:,1)));
    e_diff(:,2) = angdiff(e_jig(:,2),e_best(:,2));
    e_diff(:,3) = angdiff(e_jig(:,3),e_best(:,3));
    
    if(nargin == 7)
        set(0,'currentfigure',HP(1));
        subplot(311);
        plot_title = sprintf('Kalman estimation - It. %d', nIteration);
        title(plot_title);
        
        set(HP(2),'YData',e_best(:,1)*180/pi);
        set(HP(3),'YData',e_best(:,2)*180/pi);
        set(HP(4),'YData',e_best(:,3)*180/pi);
        drawnow;
        
        set(0,'currentfigure',HP(5));
        plot_title = sprintf('Kalman estimation error - It. %d', nIteration);
        title(plot_title);
        set(HP(6),'YData',e_diff(:,1)*180/pi);
        set(HP(7),'YData',e_diff(:,2)*180/pi);
        set(HP(8),'YData',e_diff(:,3)*180/pi);
        drawnow;
    elseif(nargin == 5)
        HP(1) = figure; 
        subplot(311);
        plot(t,e_jig(:,1)*180/pi); hold on;
        HP(2) = plot(t,e_best(:,1)*180/pi);
        ylabel('yaw [degrees]'); grid on;
        title('Kalman estimation - Initialization')
        subplot(312);
        plot(t,e_jig(:,2)*180/pi); hold on;
        HP(3) = plot(t,e_best(:,2)*180/pi);
        ylabel('roll [degrees]'); grid on;
        subplot(313);
        plot(t,e_jig(:,3)*180/pi); hold on;
        HP(4) = plot(t,e_best(:,3)*180/pi);
        ylabel('pitch [degrees]'); grid on;
        drawnow;
        
        HP(5) = figure;
        HP(6) = plot(t,e_diff(:,1)*180/pi); grid on; hold all;
        HP(7) = plot(t,e_diff(:,2)*180/pi); grid on; hold all;
        HP(8) = plot(t,e_diff(:,3)*180/pi); grid on; hold all;
        title('Kalman estimation error - Initialization');
        legend('Yaw','Roll','Pitch');
        drawnow;
    else
        error('Unacceptable number of inputs');
    end
    
    
    
    
end
end

