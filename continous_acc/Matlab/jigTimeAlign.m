function [ e_bno, e_madgwick, e_mahony, e_CF, e_gyroLib ] = jigTimeAlign( e_bno, e_madgwick, e_mahony, e_CF, e_gyroLib, e_jig )
%jigTimeAlign find the maximum cross correlation btw each signal and the
%jig orientation. Then, deslocate the signals in time domain aiming to find
%the best fit btw the sensor orientation and jig orientation
e_bno(isnan(e_bno)) = 0;
e_madgwick(isnan(e_madgwick)) = 0;
e_mahony(isnan(e_mahony)) = 0;
e_CF(isnan(e_CF)) = 0;
e_gyroLib(isnan(e_gyroLib)) = 0;

[c_bno,lag_bno]=xcorr(e_bno(:,1),e_jig(:,1));               [~,I_bno]=max(c_bno); 
[c_madgwick,lag_madgwick]=xcorr(e_madgwick(:,1),e_jig(:,1)); [~,I_madgwick]=max(c_madgwick); 
[c_mahony,lag_mahony]=xcorr(e_mahony(:,1),e_jig(:,1));      [~,I_mahony]=max(c_mahony); 
[c_CF,lag_CF]=xcorr(e_CF(:,1),e_jig(:,1));                  [~,I_CF]=max(c_CF);
[c_gyroLib,lag_gyroLib]=xcorr(e_gyroLib(:,1),e_jig(:,1));   [~,I_gyroLib]=max(c_gyroLib);

if(lag_bno(I_bno)<0) 
    e_bno = [zeros(abs(lag_bno(I_bno)),3); e_bno];
    e_bno(end+lag_bno(I_bno)+1:end,:) = [];
elseif(e_bno>0)
    e_bno = [e_bno; zeros(lag_bno(I_bno),3)];
    e_bno(1:lag_bno(I_bno),:) = [];    
end

if(lag_madgwick(I_madgwick)<0) 
    e_madgwick = [zeros(abs(lag_madgwick(I_madgwick)),3); e_madgwick];
    e_madgwick(end+lag_madgwick(I_madgwick)+1:end,:) = [];
elseif(e_madgwick>0)
    e_madgwick = [e_madgwick; zeros(lag_madgwick(I_madgwick),3)];
    e_madgwick(1:lag_madgwick(I_madgwick),:) = [];    
end

if(lag_mahony(I_mahony)<0) 
    e_mahony = [zeros(abs(lag_mahony(I_mahony)),3); e_mahony];
    e_mahony(end+lag_mahony(I_mahony)+1:end,:) = [];
elseif(e_mahony>0)
    e_mahony = [e_mahony; zeros(lag_mahony(I_mahony),3)];
    e_mahony(1:lag_mahony(I_mahony),:) = [];    
end

if(lag_CF(I_CF)<0) 
    e_CF = [zeros(abs(lag_CF(I_CF)),3); e_CF];
    e_CF(end+lag_CF(I_CF)+1:end,:) = [];
elseif(e_CF>0)
    e_CF = [e_CF; zeros(lag_CF(I_CF),3)];
    e_CF(1:lag_CF(I_CF),:) = [];    
end

if(lag_gyroLib(I_gyroLib)<0) 
    e_gyroLib = [zeros(abs(lag_gyroLib(I_gyroLib)),3); e_gyroLib];
    e_gyroLib(end+lag_gyroLib(I_gyroLib)+1:end,:) = [];
elseif(e_gyroLib>0)
    e_gyroLib = [e_gyroLib; zeros(lag_gyroLib(I_gyroLib),3)];
    e_gyroLib(1:lag_gyroLib(I_gyroLib),:) = [];    
end

end

