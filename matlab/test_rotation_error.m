clear all; close all; clc;
R=rz(2.8);
dt=1e-2;
T=3;
Kp=2;

Rd=eye(3);
 v1=[zeros(3,1) Rd(:,1)];
t=linspace(0,T,T/dt);
    e=rotationError(R,Rd);
    
    return;
for i=1:length(t)
    close all;
    de=-Kp*e;
    e=e+dt*de;
    phi=-asin(e(end)); 
    R=rz(phi);
    
    %just to check
    if (norm(e-rotationError(R,Rd)) > 1e-6)
        keyboard
    end
   
   v2=[zeros(3,1) R(:,1)];    
   v3=[zeros(3,1) e];
   plot3(0,0,0,'k.','MarkerSize',20); hold on;
plot3(v1(1,:), v1(2,:), v1(3,:),'r');
plot3(v2(1,:), v2(2,:), v2(3,:),'g');
plot3(v3(1,:), v3(2,:), v3(3,:),'b');

grid on; axis equal; rotate3d on;
xlabel('x'); ylabel('y'); zlabel('z');
axis([-1.1 1.1 -1.1 1.1 -1.1 1.1]);
view(-37.5,30);

keyboard; 

end

return;
%visualize error progression
R=eye(3);
phi=0;
v1=[0 0 0; 1 0 0]';
for i=1:16
Rd=rz(phi);
e=rotationError(R,Rd);
v2=[zeros(3,1) Rd(:,1)];
v3=[zeros(3,1) e];

plot3(0,0,0,'k.','MarkerSize',20); hold on;
plot3(v1(1,:), v1(2,:), v1(3,:),'r');
plot3(v2(1,:), v2(2,:), v2(3,:),'g');
plot3(v3(1,:), v3(2,:), v3(3,:),'b');
grid on; axis equal; rotate3d on;
xlabel('x'); ylabel('y'); zlabel('z');
axis([-1.1 1.1 -1.1 1.1 -1.1 1.1]);
view(-37.5,30);

keyboard; 
close all;
phi=phi+pi/8;
end