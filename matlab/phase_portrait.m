clear all; close all; clc;

T=5;
dt=1e-2;
Kp=1;
Kd=50;
q_ub=2;

res=linspace(-3,3,10);
count=0;
n=numel(res);
for i=1:n
    dq_0=res(1);
    q_0=res(i);
    [Q t]=simulate(T,dt, Kp, Kd, q_0, dq_0, q_ub);
    plot(Q(:,3),Q(:,2),'b'); hold on;
    dq_0=res(n);
    q_0=res(i);
    [Q t]=simulate(T,dt, Kp, Kd, q_0, dq_0, q_ub);
    plot(Q(:,3),Q(:,2),'b');
    q_0=res(1);
    dq_0=res(i);
    [Q t]=simulate(T,dt, Kp, Kd, q_0, dq_0, q_ub);
    plot(Q(:,3),Q(:,2),'b');
    q_0=res(n);
    dq_0=res(i);
    [Q t]=simulate(T,dt, Kp, Kd, q_0, dq_0, q_ub);
    plot(Q(:,3),Q(:,2),'b');
    count=count+1;
end

grid on;



