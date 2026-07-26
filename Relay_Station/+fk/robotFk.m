% Relay_Station/+fk/robotFk.m
% CR3 forward kinematics — returns 7x3 joint world positions (mm)
function joints = robotFk(j1, j2, j3, j4, j5, j6)
    d2r = pi / 180;
    j1_z=136.0; j3_x=-274.0; j4_x=-230.0; j4_z=128.3; j5_y=-116.0; j6_y=105.0;
    j2_ry=pi/2; j2_rx=pi/2; j4_rz=-pi/2; j5_rx=pi/2; j6_rx=-pi/2;
    T = eye(4); joints = zeros(7,3); joints(1,:)=[0 0 0];
    T=T*tr(0,0,j1_z)*rotz(j1*d2r); joints(2,:)=T(1:3,4)';
    T=T*roty(j2_ry)*rotx(j2_rx)*rotz(j2*d2r); joints(3,:)=T(1:3,4)';
    T=T*tr(j3_x,0,0)*rotz(j3*d2r); joints(4,:)=T(1:3,4)';
    T=T*tr(j4_x,0,j4_z)*rotz(j4_rz)*rotz(j4*d2r); joints(5,:)=T(1:3,4)';
    T=T*tr(0,j5_y,0)*rotx(j5_rx)*rotz(j5*d2r); joints(6,:)=T(1:3,4)';
    T=T*tr(0,j6_y,0)*rotx(j6_rx)*rotz(j6*d2r); joints(7,:)=T(1:3,4)';
end

function T=tr(x,y,z), T=eye(4); T(1:3,4)=[x;y;z]; end
function R=rotx(a), c=cos(a); s=sin(a); R=[1 0 0 0;0 c -s 0;0 s c 0;0 0 0 1]; end
function R=roty(a), c=cos(a); s=sin(a); R=[c 0 s 0;0 1 0 0;-s 0 c 0;0 0 0 1]; end
function R=rotz(a), c=cos(a); s=sin(a); R=[c -s 0 0;s c 0 0;0 0 1 0;0 0 0 1]; end
