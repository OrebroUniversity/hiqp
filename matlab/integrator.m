function [q_k1, dq_k1]=integrator(q_k, dq_k, ddq_k,dt)
q_k1=q_k+dt*dq_k;
dq_k1=dq_k+dt*ddq_k;
