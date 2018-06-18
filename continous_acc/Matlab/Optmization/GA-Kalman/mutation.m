 function [X] = mutation(X,n,id_best,Xmin,Xmax)
%mutation function:
%X is the cromossomic vector
%n is the number of mutations that I must do
%id_best the column of the best solution for means of elitism
%Xmin is the vector of possible minimum
%Xmax is the vector of possible maximum

P = id_best;
for k=1:n %n is the number of GA mutations
    while P==id_best
        P=round(length(X)*rand(1) + 0.5);
    end
    for j=1:size(X,1)
        M = 1-2*rand(1,1);
        if M<0.5
            X(j,P) = Xmin(j)*rand(1,1);
        elseif M>0.5
            X(j,P) = Xmax(j)*rand(1,1);
        else
            X(j,P) = X(j,P);
        end
    end
    P = id_best; %do it again with other random individual
end      %end of mutations