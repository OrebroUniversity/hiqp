function [Q t]=simulate(T,dt, Kp, Kd, q_0, dq_0, q_ub)

t=linspace(0,T,T/dt-1);
n=numel(t);
ddq_k=0;
q_k=q_0;
dq_k=dq_0;
Q=[];
E=[];
e_k=q_ub-q_0;
de_k=-dq_0;
for i=1:n
    Q=[Q; ddq_k dq_k q_k];
    E=[E; de_k e_k];
    ddq_k=PD(q_k, dq_k, q_ub, Kp, Kd);
    [q_k, dq_k]=integrator(q_k, dq_k, ddq_k,dt);
    e_k=q_ub-q_k;
    de_k=-dq_k;
end

% plot(t,Q(:,3)); grid on; hold on;
% plot(t,E(:,2),'r');
