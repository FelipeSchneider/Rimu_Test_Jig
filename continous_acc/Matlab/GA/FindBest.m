function [X,Gbest,fGbest,id_best] = FindBest(X, fit, fGbest, Gbest)
%FindBest the best particle: 
% X      - group of particles [nParameters, nParticles]
% Fit    - Fit population 
% fGbest - Fit of the best particle
% Gbest  - Genon of the best particle
%this is a elitist function 
    [new_fGbest,id] = min(fit);
    [~,idW] = max(fit);
    if new_fGbest<fGbest    %in the case there is a new best genome:
        fGbest = new_fGbest;%save the best new fit value
        Gbest = X(:,id);    %Save the best new chromosome values
        id_best = id;
    else
        X(:,idW)= Gbest;    %otherwise put the last iteration best chromosome back in the population, replacing the worst chromosome
        id_best = idW;
    end



