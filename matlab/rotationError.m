function e=rotationError(R, Rd)

e=0.5*(cross(R(:,1),Rd(:,1))+cross(R(:,2),Rd(:,2))+cross(R(:,3),Rd(:,3)));