% Relay_Station/+fk/private/tr.m
% 4x4 homogeneous translation matrix
function T = tr(x, y, z)
    T = eye(4);
    T(1:3, 4) = [x; y; z];
end
