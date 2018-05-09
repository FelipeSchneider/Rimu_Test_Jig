P=zeros(6,6);
P(1:3,1:3)=diag([1e-2, 1e-2, 1e-2]); %attitude errors in radian
P(4:6,4:6)=diag([1e-6, 1e-6, 1e-6]); %gyro bias errors in rad/s
q = [ones(length(Mb),1) zeros(length(Mb),3)];
bw = [0.01; 0.01; 0.01];
for i =1:length(Mb)-1
    fb  = Fb(i,:)'./norm(Fb(i,:));
    mb  = Mb(i,:)'./norm(Mb(i,:));
    
    [q(i+1,:), P2, bw] = ahrs_quat(q(i,:), P, bw, dWb(i,:)', fb, mb, fn, mn, dt);
end