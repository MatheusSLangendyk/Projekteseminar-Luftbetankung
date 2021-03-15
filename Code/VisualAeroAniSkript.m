%% Get Simulation DATA
out = sim('SixDOFSim','ReturnWorkspaceOutputs','on');
plane_1 = [out.Plane_1.time, out.Plane_1.Data];
plane_2 = [out.Plane_2.time, out.Plane_2.Data];

save('plane_1.mat', 'plane_1');
save('plane_2.mat', 'plane_2');

%% INIT Animation
% Creating instance of animation class
h = Aero.Animation; 

% Setting Framrate and Timescaling
h.FramesPerSecond = 10;
h.TimeScaling = 5;

% Creating Plane Objects
idx1 = h.createBody('pa24-250_orange.ac','Ac3d');
idx2 = h.createBody('pa24-250_blue.ac','Ac3d');

% Loading Simulation DATA
h.Bodies{1}.TimeSeriesSource = plane_1;
h.Bodies{2}.TimeSeriesSource = plane_2;

% Camera-Settings
h.Camera.PositionFcn = @doFirstOrderChaseCameraDynamics;
h.Camera.Offset =[-100,50,-50]/1.5;
h.Camera.ViewAngle = 25;

% Animation
h.show();
h.play();

