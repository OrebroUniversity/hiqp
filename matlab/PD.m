function ddq_k=PD(q_k, dq_k, q_ub, Kp, Kd)


e=q_ub-q_k;
de=-dq_k;

ddq_e=-Kp*e-Kd*de;
ddq_k=-ddq_e;