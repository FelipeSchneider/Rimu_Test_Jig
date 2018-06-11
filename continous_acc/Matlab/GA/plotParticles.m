function [ left_graph, right_graph ] = plotParticles(X, Xmin, Xmax, f_plot)
%Plot the particles dispertion graph
%   X   - group of particles [nParameters, nParticles]
%   Xmin - Lower limit of search [1, nParameters]
%   Xmax - Higher limit of search [1, nParameters]

if f_plot == 1
    figure(1)
    subplot(4,2,[1 3 5]); %subplot of the numerator
    left_graph = scatter3(X(1,:),X(2,:),X(3,:),'*b','linewidth',1);
    axis square;grid on;
    axis([Xmin(1) Xmax(1)*1.1 Xmin(2) Xmax(2)*1.1 Xmin(3) Xmax(3)*1.1]); %.1 prevents the 0,0 axis
    
    subplot(4,2,[2 4 6]); %subplot of the Denominator
    right_graph = scatter3(X(4,:),X(5,:),X(6,:),'*b','linewidth',1);
    axis square;grid on;
    axis([Xmin(4) Xmax(4)*1.1 Xmin(5) Xmax(5)*1.1 Xmin(6) Xmax(6)*1.1]);
end


end

