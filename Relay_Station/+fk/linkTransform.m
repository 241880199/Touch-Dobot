% Relay_Station/+fk/linkTransform.m
% CR3 per-link transform — returns 4x4 world transform for linkIdx 0..6
function T = linkTransform(j1, j2, j3, j4, j5, j6, linkIdx)
    d2r = pi / 180;
    p = params();
    T = eye(4);
    if linkIdx == 0, return; end
    T = T * tr(0, 0, p.j1_z) * rotz(j1 * d2r);
    if linkIdx == 1, return; end
    T = T * roty(p.j2_ry) * rotx(p.j2_rx) * rotz(j2 * d2r);
    if linkIdx == 2, return; end
    T = T * tr(p.j3_x, 0, 0) * rotz(j3 * d2r);
    if linkIdx == 3, return; end
    T = T * tr(p.j4_x, 0, p.j4_z) * rotz(p.j4_rz) * rotz(j4 * d2r);
    if linkIdx == 4, return; end
    T = T * tr(0, p.j5_y, 0) * rotx(p.j5_rx) * rotz(j5 * d2r);
    if linkIdx == 5, return; end
    T = T * tr(0, p.j6_y, 0) * rotx(p.j6_rx) * rotz(j6 * d2r);
end
