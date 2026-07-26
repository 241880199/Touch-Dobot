% Relay_Station/+fk/private/roty.m
% 4x4 homogeneous rotation around Y-axis
function R = roty(a)
    c = cos(a); s = sin(a);
    R = [c 0 s 0;
         0 1 0 0;
        -s 0 c 0;
         0 0 0 1];
end
