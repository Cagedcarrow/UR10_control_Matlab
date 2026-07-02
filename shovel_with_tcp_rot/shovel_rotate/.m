function viewShovelRotateControl(urdfFile)
%VIEWSHOVELROTATECONTROL Visualize shovel_rotate and control shovel angle.
%   viewShovelRotateControl opens the merged shovel_rotate URDF and provides
%   a slider for the base_tail_to_shovel_base revolute joint.

if nargin < 1 || strlength(string(urdfFile)) == 0
    scriptDir = fileparts(mfilename("fullpath"));
    urdfFile = fullfile(scriptDir, "urdf", "shovel_rotate.urdf");
end

if ~isfile(urdfFile)
    error("viewShovelRotateControl:MissingUrdf", "Cannot find URDF: %s", urdfFile);
end

packageDir = inferPackageDir(urdfFile);
model = parseUrdfModel(urdfFile, packageDir);
model.links = applyFallbackColors(model.links);

jointName = "base_tail_to_shovel_base";
jointIndex = find(string({model.joints.name}) == jointName, 1);
if isempty(jointIndex)
    error("viewShovelRotateControl:MissingJoint", ...
        "Cannot find joint %s in %s.", jointName, urdfFile);
end

fig = figure( ...
    "Name", "shovel_rotate angle control", ...
    "Color", "w", ...
    "NumberTitle", "off");

ax = axes("Parent", fig, "Position", [0.07 0.18 0.72 0.76]);
angleText = uicontrol(fig, ...
    "Style", "text", ...
    "Units", "normalized", ...
    "Position", [0.82 0.78 0.15 0.05], ...
    "BackgroundColor", "w", ...
    "FontSize", 11, ...
    "String", "0.0 deg");

uicontrol(fig, ...
    "Style", "text", ...
    "Units", "normalized", ...
    "Position", [0.80 0.70 0.18 0.04], ...
    "BackgroundColor", "w", ...
    "String", "Shovel angle");

slider = uicontrol(fig, ...
    "Style", "slider", ...
    "Units", "normalized", ...
    "Position", [0.84 0.25 0.06 0.43], ...
    "Min", -180, ...
    "Max", 180, ...
    "Value", 0, ...
    "SliderStep", [1/360 10/360]);

uicontrol(fig, ...
    "Style", "pushbutton", ...
    "Units", "normalized", ...
    "Position", [0.81 0.16 0.14 0.05], ...
    "String", "Reset", ...
    "Callback", @(~, ~) resetAngle());

slider.Callback = @(src, ~) renderModel(deg2rad(src.Value));
renderModel(0);

    function resetAngle()
        slider.Value = 0;
        renderModel(0);
    end

    function renderModel(angleRad)
        cla(ax);
        hold(ax, "on");
        axis(ax, "equal");
        grid(ax, "on");
        view(ax, 45, 25);
        xlabel(ax, "X (m)");
        ylabel(ax, "Y (m)");
        zlabel(ax, "Z (m)");
        title(ax, "shovel\_rotate: base tail to shovel base Z rotation");

        worldT = computeWorldTransforms(model, jointName, angleRad);
        for i = 1:numel(model.links)
            link = model.links(i);
            if strlength(link.meshFile) == 0
                continue;
            end
            if ~isfile(link.meshFile)
                error("viewShovelRotateControl:MissingMesh", ...
                    "Cannot find mesh for link %s: %s", link.name, link.meshFile);
            end

            mesh = stlread(link.meshFile);
            [faces, vertices] = triangulationData(mesh);
            T = worldT(char(link.name)) * link.visualOriginT;
            vertices = transformPoints(vertices, T);
            patch(ax, ...
                "Faces", faces, ...
                "Vertices", vertices, ...
                "FaceColor", link.rgba(1:3), ...
                "FaceAlpha", link.rgba(4), ...
                "EdgeColor", "none", ...
                "DisplayName", link.name);
        end

        drawLinkFrames(ax, model, worldT);
        camlight(ax, "headlight");
        lighting(ax, "gouraud");
        material(ax, "dull");
        rotate3d(fig, "on");
        angleText.String = sprintf("%.1f deg", rad2deg(angleRad));
        hold(ax, "off");
        drawnow;
    end
end

function drawLinkFrames(ax, model, worldT)
frameLength = estimateFrameLength(model, worldT);

for i = 1:numel(model.links)
    linkName = model.links(i).name;
    if ~isKey(worldT, char(linkName))
        continue;
    end

    T = worldT(char(linkName));
    origin = T(1:3, 4).';
    xEnd = origin + frameLength * T(1:3, 1).';
    yEnd = origin + frameLength * T(1:3, 2).';
    zEnd = origin + frameLength * T(1:3, 3).';

    line(ax, [origin(1), xEnd(1)], [origin(2), xEnd(2)], [origin(3), xEnd(3)], ...
        "Color", [0.86 0.10 0.10], "LineWidth", 2.0, "HitTest", "off");
    line(ax, [origin(1), yEnd(1)], [origin(2), yEnd(2)], [origin(3), yEnd(3)], ...
        "Color", [0.10 0.62 0.18], "LineWidth", 2.0, "HitTest", "off");
    line(ax, [origin(1), zEnd(1)], [origin(2), zEnd(2)], [origin(3), zEnd(3)], ...
        "Color", [0.10 0.25 0.86], "LineWidth", 2.0, "HitTest", "off");

    text(ax, origin(1), origin(2), origin(3), " " + linkName, ...
        "Interpreter", "none", ...
        "FontSize", 8, ...
        "Color", [0.08 0.08 0.08], ...
        "BackgroundColor", "w", ...
        "Margin", 1, ...
        "HitTest", "off");
end
end

function frameLength = estimateFrameLength(model, worldT)
points = zeros(0, 3);
for i = 1:numel(model.links)
    linkName = char(model.links(i).name);
    if isKey(worldT, linkName)
        T = worldT(linkName);
        points(end + 1, :) = T(1:3, 4).'; %#ok<AGROW>
    end
end

if size(points, 1) < 2
    frameLength = 0.04;
    return;
end

span = max(points, [], 1) - min(points, [], 1);
frameLength = max(norm(span) * 0.08, 0.025);
frameLength = min(frameLength, 0.08);
end

function worldT = computeWorldTransforms(model, activeJointName, activeAngle)
linkNames = string({model.links.name});
rootName = findRootLink(model.links, model.joints);
worldT = containers.Map("KeyType", "char", "ValueType", "any");
worldT(char(rootName)) = eye(4);

while worldT.Count < numel(model.links)
    madeProgress = false;
    for i = 1:numel(model.joints)
        joint = model.joints(i);
        if isKey(worldT, char(joint.parent)) && ~isKey(worldT, char(joint.child))
            jointT = joint.originT;
            if joint.name == activeJointName
                jointT = jointT * axisAngleTransform(joint.axis, activeAngle);
            end
            worldT(char(joint.child)) = worldT(char(joint.parent)) * jointT;
            madeProgress = true;
        end
    end

    if ~madeProgress
        missing = linkNames(~mapHasKeys(worldT, cellstr(linkNames)));
        error("viewShovelRotateControl:DisconnectedTree", ...
            "Cannot resolve transforms for links: %s", strjoin(missing, ", "));
    end
end
end

function T = axisAngleTransform(axisVector, angleRad)
axisVector = axisVector(:);
if norm(axisVector) == 0
    R = eye(3);
else
    axisVector = axisVector / norm(axisVector);
    x = axisVector(1);
    y = axisVector(2);
    z = axisVector(3);
    c = cos(angleRad);
    s = sin(angleRad);
    C = 1 - c;
    R = [x*x*C + c,   x*y*C - z*s, x*z*C + y*s; ...
         y*x*C + z*s, y*y*C + c,   y*z*C - x*s; ...
         z*x*C - y*s, z*y*C + x*s, z*z*C + c];
end
T = eye(4);
T(1:3, 1:3) = R;
end

function model = parseUrdfModel(urdfFile, packageDir)
doc = xmlread(urdfFile);
robot = doc.getDocumentElement();

linkNodes = robot.getElementsByTagName("link");
links = struct("name", {}, "meshFile", {}, "rgba", {}, "visualOriginT", {});

for i = 0:linkNodes.getLength() - 1
    linkNode = linkNodes.item(i);
    link.name = string(linkNode.getAttribute("name"));
    link.meshFile = "";
    link.rgba = [0.8 0.8 0.8 1];
    link.visualOriginT = eye(4);

    visualNode = firstDirectChild(linkNode, "visual");
    if ~isempty(visualNode)
        originNode = firstDirectChild(visualNode, "origin");
        link.visualOriginT = originTransform(originNode);

        geometryNode = firstDirectChild(visualNode, "geometry");
        if ~isempty(geometryNode)
            meshNode = firstDirectChild(geometryNode, "mesh");
            if ~isempty(meshNode)
                link.meshFile = resolvePackagePath( ...
                    string(meshNode.getAttribute("filename")), packageDir);
            end
        end

        materialNode = firstDirectChild(visualNode, "material");
        if ~isempty(materialNode)
            colorNode = firstDirectChild(materialNode, "color");
            if ~isempty(colorNode)
                rgba = sscanf(char(colorNode.getAttribute("rgba")), "%f").';
                if numel(rgba) == 4
                    link.rgba = rgba;
                end
            end
        end
    end

    links(end + 1) = link; %#ok<AGROW>
end

jointNodes = robot.getElementsByTagName("joint");
joints = struct("name", {}, "type", {}, "parent", {}, "child", {}, "axis", {}, "originT", {});

for i = 0:jointNodes.getLength() - 1
    jointNode = jointNodes.item(i);
    joint.name = string(jointNode.getAttribute("name"));
    joint.type = string(jointNode.getAttribute("type"));

    parentNode = firstDirectChild(jointNode, "parent");
    childNode = firstDirectChild(jointNode, "child");
    originNode = firstDirectChild(jointNode, "origin");
    axisNode = firstDirectChild(jointNode, "axis");

    joint.parent = string(parentNode.getAttribute("link"));
    joint.child = string(childNode.getAttribute("link"));
    joint.axis = [0 0 1];
    if ~isempty(axisNode) && axisNode.hasAttribute("xyz")
        parsed = sscanf(char(axisNode.getAttribute("xyz")), "%f").';
        if numel(parsed) == 3
            joint.axis = parsed;
        end
    end
    joint.originT = originTransform(originNode);
    joints(end + 1) = joint; %#ok<AGROW>
end

model.links = links;
model.joints = joints;
end

function child = firstDirectChild(parent, tagName)
child = [];
nodes = parent.getChildNodes();
for i = 0:nodes.getLength() - 1
    node = nodes.item(i);
    if node.getNodeType() == node.ELEMENT_NODE && strcmp(char(node.getNodeName()), tagName)
        child = node;
        return;
    end
end
end

function T = originTransform(originNode)
xyz = [0 0 0];
rpy = [0 0 0];

if ~isempty(originNode)
    if originNode.hasAttribute("xyz")
        parsed = sscanf(char(originNode.getAttribute("xyz")), "%f").';
        if numel(parsed) == 3
            xyz = parsed;
        end
    end
    if originNode.hasAttribute("rpy")
        parsed = sscanf(char(originNode.getAttribute("rpy")), "%f").';
        if numel(parsed) == 3
            rpy = parsed;
        end
    end
end

T = eye(4);
T(1:3, 1:3) = rpyToRotm(rpy);
T(1:3, 4) = xyz(:);
end

function R = rpyToRotm(rpy)
roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);

Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];

R = Rz * Ry * Rx;
end

function packageDir = inferPackageDir(urdfFile)
urdfDir = fileparts(urdfFile);
[~, folderName] = fileparts(urdfDir);

if strcmpi(folderName, "urdf")
    packageDir = fileparts(urdfDir);
else
    packageDir = urdfDir;
end
end

function resolved = resolvePackagePath(meshUri, packageDir)
packageName = string(getLastPathPart(packageDir));
expectedPrefix = "package://" + packageName + "/";

if startsWith(meshUri, expectedPrefix)
    relativePath = extractAfter(meshUri, strlength(expectedPrefix));
    resolved = string(fullfile(packageDir, strrep(relativePath, "/", filesep)));
elseif startsWith(meshUri, "package://")
    pathAfterScheme = extractAfter(meshUri, strlength("package://"));
    slashIndex = strfind(pathAfterScheme, "/");
    if isempty(slashIndex)
        resolved = meshUri;
        return;
    end
    relativePath = extractAfter(pathAfterScheme, slashIndex(1));
    resolved = string(fullfile(packageDir, strrep(relativePath, "/", filesep)));
else
    resolved = meshUri;
end
end

function name = getLastPathPart(pathText)
[~, name] = fileparts(char(pathText));
end

function links = applyFallbackColors(links)
fallbackColors = containers.Map( ...
    ["base_link_3", "shovel_base_link", "shovel_forward_link", ...
     "shovel_left_link", "shovel_right_link"], ...
    {[0.18 0.40 0.72 1], [0.86 0.44 0.13 1], [0.20 0.62 0.35 1], ...
     [0.72 0.25 0.55 1], [0.25 0.60 0.68 1]});

for i = 1:numel(links)
    rgba = links(i).rgba;
    needsFallback = numel(rgba) ~= 4 || rgba(4) <= 0 || all(rgba(1:3) > 0.95);
    linkName = char(links(i).name);

    if needsFallback && isKey(fallbackColors, linkName)
        links(i).rgba = fallbackColors(linkName);
    elseif needsFallback
        rgb = hsv2rgb([mod((i - 1) / max(numel(links), 1) + 0.08, 1) 0.58 0.78]);
        links(i).rgba = [rgb 1];
    end
end
end

function rootName = findRootLink(links, joints)
linkNames = string({links.name});
childNames = string({joints.child});
rootNames = setdiff(linkNames, childNames, "stable");

if numel(rootNames) ~= 1
    error("viewShovelRotateControl:InvalidRoot", ...
        "Expected exactly one root link, found %d.", numel(rootNames));
end

rootName = rootNames(1);
end

function tf = mapHasKeys(mapObj, keyList)
tf = false(size(keyList));
for i = 1:numel(keyList)
    tf(i) = isKey(mapObj, keyList{i});
end
end

function [faces, vertices] = triangulationData(mesh)
if isa(mesh, "triangulation")
    faces = mesh.ConnectivityList;
    vertices = mesh.Points;
elseif isstruct(mesh) && isfield(mesh, "ConnectivityList") && isfield(mesh, "Points")
    faces = mesh.ConnectivityList;
    vertices = mesh.Points;
elseif isstruct(mesh) && isfield(mesh, "faces") && isfield(mesh, "vertices")
    faces = mesh.faces;
    vertices = mesh.vertices;
else
    error("viewShovelRotateControl:UnsupportedStlReadOutput", ...
        "Unsupported stlread output type: %s", class(mesh));
end
end

function vertices = transformPoints(vertices, T)
verticesHomogeneous = [vertices, ones(size(vertices, 1), 1)] * T.';
vertices = verticesHomogeneous(:, 1:3);
end
