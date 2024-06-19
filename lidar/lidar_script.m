clear all
close all
clc

%% SCENARIO
% Initialize the scenario 
scene = uavScenario(UpdateRate=100,ReferenceLocation=[0 0 0]);

% Create a ground for visualization
addMesh(scene,"polygon",{[-15 -15; 15 -15; 15 15; -15 15] [-0.5 0]},[0.3 0.3 0.3]);

% Add cylinder meshes to scan with lidar sensor
addMesh(scene,"cylinder",{[-5 5 2],[0 12]},[0 1 0]);
addMesh(scene,"cylinder",{[5 5 2],[0 12]},[0 1 0]);
addMesh(scene,"cylinder",{[5 -5 2],[0 12]},[0 1 0]);


%% UAV 
% Platform/UAV initial position and orientation
initpos = [0 0 -5]; % NED Frame
initori = [0 0 0];

% Add UAV Platform to the scenario and scale it for easier visualization
platform = uavPlatform("platformUAV",scene,ReferenceFrame="NED",...
    InitialPosition=initpos,InitialOrientation=eul2quat(initori));

updateMesh(platform,"quadrotor",{2},[0 0 0],eul2tform([0 0 pi]));

% Add Lidar sensor to UAV
LidarModel = uavLidarPointCloudGenerator();
uavSensor("Lidar",platform,LidarModel,"MountingLocation",[0,0,1],"MountingAngles",[0 0 180]);


open_system("UAV_Model.slx")

%% POINTS CLOUD
firstPtCloud = pointCloud(out.nuvola.Data(:,:,:,1));
allPtsCloud = firstPtCloud;

for i = 2:length(out.tout)
    allPtsCloud = [allPtsCloud pointCloud(out.nuvola.Data(:,:,:,i))];
end

pcshow(firstPtCloud);      

%[indices,dists] = findNeighborsInRadius(singleptcloud,[0 0 -5],10);
[indices,dists] = findNearestNeighbors(firstPtCloud,[0 0 -5],100);

newptCloudOut = select(firstPtCloud,indices);

% Posizione q_obstacle rispetto alla mappa 
vicinissimo = newptCloudOut.Location(1,:);

% Distanza rispetto all'origine
norm(vicinissimo);

% Distanza rispetto al drone nell'istante in cui è generata la nuvola di punti
norm(vicinissimo-[0 0 -5]); 
pcshow(newptCloudOut);

