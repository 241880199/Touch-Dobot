function fk_validate(inputFile, outputFile)
    % FK_VALIDATE  Standalone FK validation — reads joint configs from JSON, writes results to JSON.
    %   fk_validate('input.json', 'matlab_output.json')
    %
    %   Uses the EXACT same computeFK as relay_gui.m (lines 375-437) for apples-to-apples comparison.
    %
    %   Input JSON:  {"configs": [{"label": "all_zeros", "joints": [0,0,0,0,0,0]}, ...]}
    %   Output JSON: {"configs": [{"label": "...", "input": [...], "joint_positions": [7x3], "ee_position": [1x3]}]}

    if nargin < 1, inputFile = 'input.json'; end
    if nargin < 2, outputFile = 'matlab_output.json'; end

    data = jsondecode(fileread(inputFile));
    results.configs = struct([]);

    for i = 1:length(data.configs)
        c = data.configs(i);
        j = c.joints;
        joint_pos = computeFK(j(1), j(2), j(3), j(4), j(5), j(6));

        results.configs(i).label = c.label;
        results.configs(i).input = j(:)';
        results.configs(i).joint_positions = joint_pos;
        results.configs(i).ee_position = joint_pos(7, :);
    end

    fid = fopen(outputFile, 'w');
    fprintf(fid, '%s', jsonencode(results, 'PrettyPrint', true));
    fclose(fid);

    fprintf('fk_validate: %d configs → %s\n', length(data.configs), outputFile);
end

% ========================================================================
%  CR3 Forward Kinematics — copied verbatim from relay_gui.m lines 375-437
%  DO NOT MODIFY these functions. They must stay identical to relay_gui.m.
% ========================================================================

function joints = computeFK(j1, j2, j3, j4, j5, j6)
    % CR3 Forward Kinematics (URDF params, angles in degrees, positions in mm)
    d2r = pi / 180;

    % URDF joint parameters (mm, radians)
    j1_z  = 128.3;
    j3_x  = -274.0;
    j4_x  = -230.0;  j4_z = 128.3;
    j5_y  = -116.0;
    j6_y  = 105.0;

    % rpy fixed rotations (URDF fixed-axis: R = Rz(yaw) * Ry(pitch) * Rx(roll))
    j2_ry = pi/2;  j2_rx = pi/2;   % rpy="1.5708 1.5708 0" → Ry(π/2)*Rx(π/2)
    j4_rz = -pi/2;                  % rpy="0 0 -1.5708" → Rz(-π/2)
    j5_rx = pi/2;                   % rpy="1.5708 0 0" → Rx(π/2)
    j6_rx = -pi/2;                  % rpy="-1.5708 0 0" → Rx(-π/2)

    % 4x4 homogeneous transform
    T = eye(4);
    joints = zeros(7, 3);
    joints(1,:) = [0 0 0];  % base origin

    % J1: base → Link1
    T = T * tr(0, 0, j1_z) * rotz(j1 * d2r);
    joints(2,:) = T(1:3,4)';

    % J2: Link1 → Link2
    T = T * roty(j2_ry) * rotx(j2_rx) * rotz(j2 * d2r);
    joints(3,:) = T(1:3,4)';

    % J3: Link2 → Link3
    T = T * tr(j3_x, 0, 0) * rotz(j3 * d2r);
    joints(4,:) = T(1:3,4)';

    % J4: Link3 → Link4
    T = T * tr(j4_x, 0, j4_z) * rotz(j4_rz) * rotz(j4 * d2r);
    joints(5,:) = T(1:3,4)';

    % J5: Link4 → Link5
    T = T * tr(0, j5_y, 0) * rotx(j5_rx) * rotz(j5 * d2r);
    joints(6,:) = T(1:3,4)';

    % J6: Link5 → Link6
    T = T * tr(0, j6_y, 0) * rotx(j6_rx) * rotz(j6 * d2r);
    joints(7,:) = T(1:3,4)';
end

% Helper: homogeneous translation
function T = tr(x, y, z)
    T = eye(4); T(1:3,4) = [x; y; z];
end

function R = rotx(angle)
    c = cos(angle); s = sin(angle);
    R = [1 0 0 0; 0 c -s 0; 0 s c 0; 0 0 0 1];
end

function R = roty(angle)
    c = cos(angle); s = sin(angle);
    R = [c 0 s 0; 0 1 0 0; -s 0 c 0; 0 0 0 1];
end

function R = rotz(angle)
    c = cos(angle); s = sin(angle);
    R = [c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1];
end
