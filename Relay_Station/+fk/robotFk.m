% Relay_Station/+fk/robotFk.m
% CR3 forward kinematics — returns 7x3 joint world positions (mm)
function joints = robotFk(j1, j2, j3, j4, j5, j6)
    d2r = pi / 180;
    p = params();
    T = eye(4);
    joints = zeros(7, 3);
    joints(1, :) = [0 0 0];
    T = T * tr(0, 0, p.j1_z) * rotz(j1 * d2r);
    joints(2, :) = T(1:3, 4)';
    T = T * roty(p.j2_ry) * rotx(p.j2_rx) * rotz(j2 * d2r);
    joints(3, :) = T(1:3, 4)';
    T = T * tr(p.j3_x, 0, 0) * rotz(j3 * d2r);
    joints(4, :) = T(1:3, 4)';
    T = T * tr(p.j4_x, 0, p.j4_z) * rotz(p.j4_rz) * rotz(j4 * d2r);
    joints(5, :) = T(1:3, 4)';
    T = T * tr(0, p.j5_y, 0) * rotx(p.j5_rx) * rotz(j5 * d2r);
    joints(6, :) = T(1:3, 4)';
    T = T * tr(0, p.j6_y, 0) * rotx(p.j6_rx) * rotz(j6 * d2r);
    joints(7, :) = T(1:3, 4)';
end
