% Relay_Station/+fk/private/rotx.m
% 4x4 homogeneous rotation around X-axis
function R = rotx(a)
    c = cos(a); s = sin(a);
    R = [1 0  0 0;
         0 c -s 0;
         0 s  c 0;
         0 0  0 1];
end
