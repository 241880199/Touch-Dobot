function mesh = loadBinaryStl(filepath)
% LOADBINARYSTL 读取二进制 STL 文件
%   mesh = stl.loadBinaryStl(filepath)
%   返回 struct:
%     .vertices  — (N×3) 顶点坐标 (mm, 从 m 缩放)
%     .faces     — (M×3) 面索引 (1-based)
%     .normals   — (M×3) 面法线
%     .triangleCount — 三角面总数
    mesh = struct('vertices', [], 'faces', [], 'normals', [], 'triangleCount', 0);
    if ~isfile(filepath)
        warning('stl:fileNotFound', 'STL file not found: %s', filepath);
        return;
    end
    fid = fopen(filepath, 'rb');
    if fid < 0, warning('stl:openFailed', 'Cannot open: %s', filepath); return; end
    fseek(fid, 80, 'bof');
    count = fread(fid, 1, 'uint32');
    if isempty(count) || count == 0, fclose(fid); return; end
    raw = fread(fid, count * 12, 'float32');
    fclose(fid);
    if numel(raw) < count * 12, warning('stl:truncated', 'File truncated'); return; end
    raw = reshape(raw, 12, count)';
    mesh.normals = raw(:, 1:3);
    vertData = raw(:, 4:12)';
    mesh.vertices = reshape(vertData, 3, count*3)' * 1000;  % m → mm
    mesh.faces = reshape(1:(count*3), 3, count)';
    mesh.triangleCount = count;
end
