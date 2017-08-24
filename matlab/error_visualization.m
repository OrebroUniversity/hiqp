clear all; close all; clc;

a1=[  -0.981994   0.180508 -0.0557247]';
a2=[ 0.0750897  0.643627  0.761647]';
a3=[   0.17335  0.743748 -0.645592]';

R=rz(1);
a1=R(:,1); a2=R(:,2); a3=R(:,3);

b1=[  0.954808 -0.297224         0]';
b2=[ 0.297224 0.954808       -0]';
b3=[0 0 1]';
Rd=[b1 b2 b3]';
dR=R'*Rd;
dq=R2q(dR);
[an ax]=R2aa(dR);

e=0.5*(cross(a1,b1)+cross(a2,b2)+cross(a3,b3));
n=e/norm(e)*1.1;
phi=asin(e(end))*180/pi;
angle=an*180/pi

plot3(0,0,0,'k.','MarkerSize',10);  hold on;
plot3([0 a1(1)],[0 a1(2)], [0 a1(3)],'r');
plot3([0 a2(1)],[0 a2(2)], [0 a2(3)],'g');
plot3([0 a3(1)],[0 a3(2)], [0 a3(3)],'b');

plot3([0 ax(1)]*1.1,[0 ax(2)]*1.1, [0 ax(3)]*1.1,'k','LineWidth',2);

plot3([0 b1(1)],[0 b1(2)], [0 b1(3)],'r','LineWidth',5);
plot3([0 b2(1)],[0 b2(2)], [0 b2(3)],'g','LineWidth',5);
plot3([0 b3(1)],[0 b3(2)], [0 b3(3)],'b','LineWidth',5);

grid on; axis equal; rotate3d on;
xlabel('x'); ylabel('y'); zlabel('z');
axis([-1.1 1.1 -1.1 1.1 -1.1 1.1]);
view(-210,50);