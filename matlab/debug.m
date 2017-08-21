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
%load unfiltered.mat

u=u(:,1:6);
dJ=dJ(:,1:6);
J=J(:,1:6);
q=q(:,1:6);
dq=dq(:,1:6);

dt=1e-3;
n=size(u,1);
t=linspace(0,n*dt,n)';

% ind_s=6772;
% ind_e=7000;
% u=u(ind_s:ind_e,:);
% dJ=dJ(ind_s:ind_e,:);
% J=J(ind_s:ind_e,:);
% q=q(ind_s:ind_e,:);
% dq=dq(ind_s:ind_e,:);
% e=e(ind_s:ind_e);
% de=de(ind_s:ind_e);
% dde_star=dde_star(ind_s:ind_e);
% t=t(ind_s:ind_e);

% system definition
SP = model_UR10();
SV = System_Variables(SP);

%create an instance of the visualizer
%visualizer=MBSVisualizer(SP,SV); 
% axis([-1.2 1.2 -1.2 1.2 -0.4 1.2]);
% pbaspect([1 1 1]);
subplot(1,3,1);
plot(t,e,'b'); grid on; hold on;
subplot(1,3,2);
plot(t,de,'r'); grid on; hold on;
subplot(1,3,3);
plot(t,dde_star,'m'); grid on; hold on;

e_=e(1);
de_=de(1);
Kp=6;
Kd=9;

%comute ideal response
for i=1:length(e)
   dde_star_(i)=-Kp*e_(i)-Kd*de_(i);
   de_(i+1)=de_(i)+dt*dde_star_(i);
   e_(i+1)=e_(i)+dt*de_(i);
end    
e_(end)=[];
de_(end)=[];

subplot(1,3,1);
plot(t,e_,'k'); grid on; 
subplot(1,3,2);
plot(t,de_,'k'); grid on;
subplot(1,3,3);
plot(t,dde_star_,'k'); grid on;

%compute ideal response full


% ind_s=4925;
% ind_e=4935;
% figure;
% t_=1:1:ind_e-ind_s+1';
% q_=q(ind_s:ind_e,1);
% dq_=dq(ind_s:ind_e,1);
% de_=de(ind_s:ind_e);
% dde_star_=dde_star(ind_s:ind_e);
% de_des=de_(1);
% for i=2:length(de_)
%        de_des(i)=de_(i-1)+dde_star_(i-1); 
% end
% 
% plot(t_,e_,'b'); grid on; hold on;
% plot(t_,de_,'r');
% plot(t_,dde_star_,'m');
% plot(t_,de_des,'r*');
% legend('e','de','dde^*','de_des');
% xlim([1 ind_e-ind_s+1]);
%%%EOF
