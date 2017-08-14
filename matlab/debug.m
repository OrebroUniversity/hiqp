% uses function normalize_angles.m

clear all; close all; clc
%load debug data
load dde_star.dat;
load de.dat;
load e.dat;
load dJ.dat
load J.dat;
load dq.dat;
load q.dat;
load u.dat;
u=u(:,1:6);
dJ=dJ(:,1:6);
J=J(:,1:6);
q=q(:,1:6);
dq=dq(:,1:6);

dt=1e-3;
n=size(u,1);
t=linspace(0,n*dt,n)';

% system definition
SP = model_UR10();
SV = System_Variables(SP);

%create an instance of the visualizer
%visualizer=MBSVisualizer(SP,SV); 
% axis([-1.2 1.2 -1.2 1.2 -0.4 1.2]);
% pbaspect([1 1 1]);
plot(t,e,'b'); grid on; hold on;
plot(t,de,'r');
%plot(t,dde_star,'m');
legend('e','de','dde^*');
return
ind_s=4925;
ind_e=4935;

figure;
t_=1:1:ind_e-ind_s+1';
q_=q(ind_s:ind_e,1);
dq_=dq(ind_s:ind_e,1);


de_=de(ind_s:ind_e);
dde_star_=dde_star(ind_s:ind_e);

de_des=de_(1);
for i=2:length(de_)
       de_des(i)=de_(i-1)+dde_star_(i-1); 
end



plot(t_,e_,'b'); grid on; hold on;
plot(t_,de_,'r');
plot(t_,dde_star_,'m');
plot(t_,de_des,'r*');
legend('e','de','dde^*','de_des');
xlim([1 ind_e-ind_s+1]);
% for i=1:n
% SV.q=q(i,:)';
% SV.dq=dq(i,:)';
% 
% % updates positions & velocities of links
% SV = calc_pos(SP,SV); 
% SV = calc_vel(SP,SV); 
% 
% %forward kinematics & end-effector Jacobian
% [pE,RE] = fk_e(SP,SV,SP.bN,SP.bP,SP.bR);
% Je = calc_Je(SP,SV,SP.bN,SP.bP); 
% 
% [dJe,joints] = calc_dJe(SP,SV,SP.bN,SP.bP);
% %Jacobian derivative
% 
%  i
% end
% rotate3d on;
%%%EOF
