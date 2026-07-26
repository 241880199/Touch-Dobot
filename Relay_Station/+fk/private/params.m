% Relay_Station/+fk/private/params.m
% FK parameters struct — shared by robotFk and linkTransform
function p = params()
    p = struct();
    p.j1_z = 136.0;  p.j3_x = -274.0;
    p.j4_x = -230.0; p.j4_z = 128.3;
    p.j5_y = -116.0; p.j6_y = 105.0;
    p.j2_ry = pi/2;  p.j2_rx = pi/2;
    p.j4_rz = -pi/2;
    p.j5_rx = pi/2;
    p.j6_rx = -pi/2;
end
