function [ HP ] = plotParticles(X, Gbest, Xmin, Xmax, f_plot, nIteration, HP)
%Plot the particles dispertion graph
%   X   - group of particles [nParameters, nParticles]
%   Xmin - Lower limit of search [1, nParameters]
%   Xmax - Higher limit of search [1, nParameters]
%   f_plot - flag that will determine if we plot the graphs or not
%   nIteration - number of the iteration to that will apear in the title
%   HP - Handle plot of the graph


if f_plot == 1
    if(nargin == 7)
        set(HP(1),'XData',X(1,:));
        set(HP(1),'Ydata',X(2,:));
        set(HP(1),'Zdata',X(3,:));
        set(HP(2),'XData',Gbest(1));
        set(HP(2),'YData',Gbest(2));
        set(HP(2),'ZData',Gbest(3));
        
        set(HP(3),'XData',X(4,:));
        set(HP(3),'Ydata',X(5,:));
        set(HP(3),'Zdata',X(6,:));
        set(HP(4),'XData',Gbest(4));
        set(HP(4),'YData',Gbest(5));
        set(HP(4),'ZData',Gbest(6));
        
        set(0,'currentfigure',HP(5));
        plot_title = sprintf('Particle distribution - It. %d', nIteration);
        suptitle(plot_title);
    elseif(nargin == 5)
        HP(5) = figure;
        subplot(4,2,[1 3 5]); %subplot of the numerator
        HP(1) = scatter3(X(1,:),X(2,:),X(3,:),'*b','linewidth',1); hold on;
        HP(2) = scatter3(Gbest(1),Gbest(2),Gbest(3),'or','linewidth',3);
        axis square;grid on;
        axis([Xmin(1) Xmax(1)*1.1 Xmin(2) Xmax(2)*1.1 Xmin(3) Xmax(3)*1.1]); %.1 prevents the 0,0 axis
        xlabel('X_1');ylabel('X_2');zlabel('X_3');
        
        subplot(4,2,[2 4 6]); %subplot of the Denominator
        HP(3) = scatter3(X(4,:),X(5,:),X(6,:),'*b','linewidth',1); hold on;
        HP(4) = scatter3(Gbest(4),Gbest(5),Gbest(6),'or','linewidth',3);
        axis square;grid on;
        axis([Xmin(4) Xmax(4)*1.1 Xmin(5) Xmax(5)*1.1 Xmin(6) Xmax(6)*1.1]);
        xlabel('X_4');ylabel('X_5');zlabel('X_6');
        suptitle('Particle distribution - Initialization');      
    else
        error('Unacceptable number of inputs');
    end
    drawnow;
end


end

