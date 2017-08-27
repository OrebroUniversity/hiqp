clear all; close all; clc;

phi=3*pi/4;
phi_max=pi/2+0.2;

v1=[1 0 0]';
R=rz(phi);
v2=R*v1;

n=cross(v1,v2); n=n/norm(n);
v2_=v1*cos(phi_max)+cross(n,v1)*sin(phi_max)+n*(n'*v1)*(1-cos(phi_max));


plot3(0,0,0,'k.','MarkerSize',20); hold on;
plot3([v1(1) 0], [v1(2) 0], [v1(3) 0],'b');
plot3([v2(1) 0], [v2(2) 0], [v2(3) 0],'r');
plot3([n(1) 0], [n(2) 0], [n(3) 0],'k','LineWidth',2);
plot3([v2_(1) 0], [v2_(2) 0], [v2_(3) 0],'m');

grid on; axis equal; rotate3d on;
xlabel('x'); ylabel('y'); zlabel('z');
axis([-1.1 1.1 -1.1 1.1 -1.1 1.1]);
view(-37.5,30);

phi_max
 acos(v1'*v2_)