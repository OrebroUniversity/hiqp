clear all; close all; clc;

T=5;
dt=1e-3;
Kp=2;
Kd=3;
q_k=1.99776;
q_k=1.9;
dq_k=1.8375;
q_ub=2;

t=linspace(0,T,T/dt-1);
n=numel(t);
ddq_k=0;
Q=[];
E=[];
e_k=q_ub-q_k;
de_k=-dq_k;
for i=1:n
    Q=[Q; ddq_k dq_k q_k];
    E=[E; de_k e_k];
    ddq_k=PD(q_k, dq_k, q_ub, Kp, Kd);
    [q_k, dq_k]=integrator(q_k, dq_k, ddq_k,dt);
    e_k=q_ub-q_k;
    de_k=-dq_k;
end

plot(t,Q(:,3)); grid on; hold on;
plot(t,E(:,2),'r');
