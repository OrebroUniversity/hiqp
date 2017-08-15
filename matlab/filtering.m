% uses function normalize_angles.m

clear all; close all; clc
%load debug data
% load dde_star.dat;
% load de.dat;
% load e.dat;
% load dJ.dat
% load J.dat;
% load dq.dat;
% load q.dat;
% load u.dat;

load unfiltered.mat;

u=u(:,1:6);
dJ=dJ(:,1:6);
J=J(:,1:6);
q=q(:,1:6);
dq=dq(:,1:6);

dt=1e-3;
n=size(u,1);
t=linspace(0,n*dt,n)';
Kp=2;
Kd=3;
pos_a=[1 -0.01];
pos_b=[0.99];
vel_a=[1 -0.995];
vel_b=[0.005];

[vel_b, vel_a] = butter(3,0.1);  
%[pos_b, pos_a] = butter(3,0.05);  

e_f=filter(pos_b,pos_a,e);
de_f=filter(vel_b,vel_a,de);
dde_star_f=-Kp*e_f-Kd*de_f;

subplot(1,3,1);
plot(t,e,'b'); grid on; hold on;
plot(t,e_f,'k');
subplot(1,3,2);
plot(t,de,'r'); grid on; hold on;
plot(t,de_f,'k');
subplot(1,3,3);
plot(t,dde_star,'m'); grid on; hold on;
plot(t,dde_star_f,'k'); 

norm(diff(dde_star_f))