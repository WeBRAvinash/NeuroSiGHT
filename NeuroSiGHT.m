clc; clear; close all; 

import org.opensim.modeling.*;

modelPath = fullfile(erase(pwd, 'Functions\neurosight'), 'Models', 'OpenSim_Model', 'Hand_Wrist_Model.osim');
model = Model(modelPath);
state = model.initSystem();

dofs= {'deviation', 'flexion'};
coordRefs = containers.Map();
targetBodies = {'2distph', 'distal_thumb', '3distph'};

dstncs = [];
params.w_proprio = 0.6;
params.w_haptic = 0.3;
params.w_visual = 0.1;
params.perception_noise = 20; % Gaussian noise standard deviation
params.torque_noise = 25;

params.skill = [100 -500];
params.skillmax = [1 1];
params.eta = 0.1;   % Increase learning rate (was 0.1)
params.beta = 0.2;  % Increase beta (was 0.2) for faster changes
params.P = 2;       % Increase external factor influence (was 1)
params.mk = 1;
params.mk_1 = 0;
params.skillhistory = [];
skills = [];

for k = 1:2
params.mk = not(params.mk_1);
params.skill = skillAscent(params.skill,params.skillmax, params.eta, params.beta,params.mk,params.mk_1);


params.mk_1 = params.mk;
params.accuracyGain= params.skill(1);
params.precisionGain = params.skill(2);
params.skillhistory = [params.skillhistory; params.skill];


seed1 = [-randi(20,1,10)];
x1 = []; x = [];
for i = 1:9
    x1 = [x1; genTrajs([seed1(i),1], seed1(i+1),params)];
end
seed2 = [-randi(20,1,5) randi(20,1,5)];
for i = 1:9
    x = [x; genTrajs([seed2(i),1], seed2(i+1),params)];
end

wristAngles_x1 = downsample(x1,10)*5;
wristAngles_x2 = downsample(x,10)*5;
numTimeSteps = length(wristAngles_x1);
seed1 = seed1*5;
seed2 = seed2*5;
%
[position,ArmConfiguration] = getPositions([],[wristAngles_x1 wristAngles_x2], model, state, coordRefs, dofs, numTimeSteps, targetBodies);
numTimeSteps = length(seed1);
[points,ArmConfiguration1] = getPositions([],[seed1' seed2'], model, state, coordRefs, dofs, numTimeSteps, targetBodies);

% Define body groups and connections between bodies
armBodies = {'clavicle', 'scapula', 'humerus', 'ulna', 'radius', 'proximal_row', 'capitate', ...
             'trapezium', 'trapezoid', 'hamate'};
thumbBodies = {'firstmc1', 'firstmc', 'proximal_thumb', 'distal_thumb'};
indexBodies = {'secondmc', '2proxph', '2midph', '2distph'};
middleBodies = {'thirdmc', '3proxph', '3midph', '3distph'};
ringBodies = {'fourthmc', '4proxph', '4midph', '4distph'};
pinkyBodies = {'fifthmc', '5proxph', '5midph', '5distph'};

connections = { ... % Define connections between bodies
    'clavicle', 'scapula'; 'scapula', 'humerus'; 'humerus', 'ulna'; 
    'ulna', 'radius'; 'radius', 'proximal_row'; 'proximal_row', 'capitate'; 
    'capitate', 'trapezium'; 'trapezium', 'firstmc1'; 'firstmc1', 'firstmc'; 
    'firstmc', 'proximal_thumb'; 'proximal_thumb', 'distal_thumb';
    'trapezoid', 'secondmc'; 'secondmc', '2proxph'; '2proxph', '2midph'; 
    '2midph', '2distph'; 'capitate', 'thirdmc'; 'thirdmc', '3proxph'; 
    '3proxph', '3midph'; '3midph', '3distph'; 'hamate', 'fourthmc'; 
    'fourthmc', '4proxph'; '4proxph', '4midph'; '4midph', '4distph';
    'fifthmc', '5proxph'; '5proxph', '5midph'; '5midph', '5distph'
};

% Define colors for each part
colors = containers.Map;
colors('arm') = [0, 0, 0];
colors('thumb') = [1, 0, 0];
colors('index') = [0, 1, 0];
colors('middle') = [1, 0.5, 0];
colors('ring') = [0.5, 0, 0.5];
colors('pinky') = [0, 1, 1];

% Determine the total number of frames
numFrames = size(ArmConfiguration.body_clavicle, 1);

% Set up the figure
close all;
figure('Position', [50, 50, 600, 400]);
hold on;
grid on;
xlabel('X Position');
ylabel('Y Position');
zlabel('Z Position');
title('3D Animated Arm Trajectory with Endpoint-to-Finger Connection');
view(70, 40);

vects = [sin(wristAngles_x1)  cos(wristAngles_x1) cos(wristAngles_x2)]/10+[.264 -.266 0.180];
vects = [cosd(seed2'), sind(seed2'), cosd(seed1')] / 10+[.264 -.266 0.180];

% Animation loop for each time frame
for t = 1:numFrames
    % Plot each body group
    arm_plot = plotBodyGroup(armBodies, colors('arm'), ArmConfiguration, t);
    thumb_plot = plotBodyGroup(thumbBodies, colors('thumb'), ArmConfiguration, t);
    index_plot = plotBodyGroup(indexBodies, colors('index'), ArmConfiguration, t);
    middle_plot = plotBodyGroup(middleBodies, colors('middle'), ArmConfiguration, t);
    ring_plot = plotBodyGroup(ringBodies, colors('ring'), ArmConfiguration, t);
    pinky_plot = plotBodyGroup(pinkyBodies, colors('pinky'), ArmConfiguration, t);

    % Plot connections between body segments
    conn_plot = gobjects(size(connections, 1), 1);
    for c = 1:size(connections, 1)
        body1 = connections{c, 1};
        body2 = connections{c, 2};
        conn_plot(c) = plotConnection(ArmConfiguration, body1, body2, t);
    end

    xlim([-0.1 0.55]);
    ylim([-0.4 0.1]);
    zlim([0 0.3]);
    pause(0.01);
    x = ArmConfiguration.body_2distph(t,:);
    s = .6;
    L_wrist_to_index = 0.1; % Approximate length from wrist to index tip
    vectplot = plot3(s, ...
          x(2), ...
          x(3), '.m', 'LineWidth', 2);    
    xlim([-0.1 s]);
    ylim([-0.4 0.1]);
    zlim([0 .3]);
    pause(0.01);
    
%    Clear previous plots to make magenta line transient
    if t < numTimeSteps
        delete([arm_plot(:); thumb_plot(:); index_plot(:); conn_plot(:);]);
    end

    % Clear the plot objects after each time frame to prevent overdraw
    if t < numFrames
        delete([arm_plot(:); thumb_plot(:); index_plot(:); middle_plot(:); ring_plot(:); pinky_plot(:); conn_plot(:)]);
    end
    points(:,1) = s*ones(length(points),1);
    plot3(points(:,1),...
        points(:,2),...
        points(:,3),'ko')
    
    
end

    % Preallocate array to store minimum distances and indices
    minDistances = zeros(size(points, 2), 1);
    closestIndices = zeros(size(points, 2), 1);
    
    % Compute minimum distance for each point in A from all points in B
    for i = 1:size(points, 2)
        distances = vecnorm(x(:,2) - points(:,i), 2, 2); % Euclidean distance
        [minDistances(i), closestIndices(i)] = min(distances);
    end
end


%% Helper function to plot each body group
function plot_handles = plotBodyGroup(group, color, ArmConfiguration, t)
    plot_handles = gobjects(1, length(group));
    for b = 1:length(group)
        bodyName = group{b};
        fieldName = matlab.lang.makeValidName(['body_', bodyName]);
        if isfield(ArmConfiguration, fieldName) && t <= size(ArmConfiguration.(fieldName), 1)
            plot_handles(b) = plot3(ArmConfiguration.(fieldName)(t, 1), ...
                                    ArmConfiguration.(fieldName)(t, 2), ...
                                    ArmConfiguration.(fieldName)(t, 3), ...
                                    '.', 'MarkerSize', 10, 'Color', color);
        end
    end
end

%% Helper function to plot connection lines
function conn_plot = plotConnection(ArmConfiguration, body1, body2, t)
    conn_plot = gobjects(1);
    field1 = matlab.lang.makeValidName(['body_', body1]);
    field2 = matlab.lang.makeValidName(['body_', body2]);
    if isfield(ArmConfiguration, field1) && isfield(ArmConfiguration, field2)
        conn_plot = plot3([ArmConfiguration.(field1)(t,1), ArmConfiguration.(field2)(t,1)], ...
                          [ArmConfiguration.(field1)(t,2), ArmConfiguration.(field2)(t,2)], ...
                          [ArmConfiguration.(field1)(t,3), ArmConfiguration.(field2)(t,3)], ...
                          'k-', 'LineWidth', 1);
    end
end


%%