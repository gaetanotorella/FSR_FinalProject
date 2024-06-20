clear all
close all
clc

%% TRIGGER
video = 0;

%% UAV PARAMETERS
mass = 1.2;
Ib = diag([1.2416 1.2416 2*1.2416]);

%% GEOMETRIC CONTROLLER PARAM
Ts = 0.01;
u0_t = 11.77;
g = 9.81;

kp = diag([1 1 1]);
kv = diag([1 1 1]);
kr = diag([1 1 1]);
kw = diag([1 1 1]);

% kp = diag([100 160 175]);
% kv = diag([10 10 20]);
% kr = diag([150 150 170]);
% kw = diag([50 50 50]);

%% ARTIFICIAL POTENTIAL PARAM
k_att = 2.0;
k_rep = 0.2;
d0 = 2.0;

%% SCENARIO
% Initialize the scenario 
scene = uavScenario(UpdateRate=1/Ts,ReferenceLocation=[0 0 0]);

% Create a ground for visualization
addMesh(scene,"polygon",{[-15 -15; 15 -15; 15 15; -15 15] [-0.5 0]},[0.3 0.3 0.3]);

% Add cylinder meshes to scan with lidar sensor
%addMesh(scene,"cylinder",{[-5 5 0.5],[0 10]},[1 0 0]);
%addMesh(scene,"cylinder",{[3 6 0.5],[0 10]},[0 1 0]);
%addMesh(scene,"cylinder",{[5 -5 0.5],[0 10]},[0 0 1]);
% (scenario, "oggetto", {[x y radius], [boh altezza], [colore]}

%% UAV 
% Platform/UAV initial position and orientation
initpos = [0 0 -5]; % NED Frame
initori = [0 0 0];
initvel = [0 0 0];
initacc = [0 0 0];

q_goal = [9 9 -5];

% Add UAV Platform to the scenario and scale it for easier visualization
platform = uavPlatform("platformUAV",scene,ReferenceFrame="NED",...
    InitialPosition=initpos,InitialOrientation=eul2quat(initori));

updateMesh(platform,"quadrotor",{2},[0 0 0],eul2tform([0 0 pi]));

% Add Lidar sensor to UAV
LidarModel = uavLidarPointCloudGenerator("UpdateRate",1/Ts);
uavSensor("Lidar",platform,LidarModel,"MountingLocation",[0,0,1],"MountingAngles",[0 0 180]);


open_system("UAV_Model.slx")

%% VISUAL
if video == 1
    figure()
    plot3(out.pos_dy.Data(1,1),out.pos_dy.Data(1,2),out.pos_dy.Data(1,3),"o",'Color',"b",'MarkerSize',10)
    hold on
    for i = 1:length(out.u_t.Data)
        plot3(out.pos_dy.Data(:,1),out.pos_dy.Data(:,2),out.pos_dy.Data(:,3))
    end
    grid on

end

%% POINTS CLOUD
% firstPtCloud = pointCloud(out.nuvola.Data(:,:,:,1));
% allPtsCloud = firstPtCloud;
% 
% for i = 2:length(out.tout)
%     allPtsCloud = [allPtsCloud pointCloud(out.nuvola.Data(:,:,:,i))];
% end
% 
% pcshow(firstPtCloud);      
% 
% %[indices,dists] = findNeighborsInRadius(singleptcloud,[0 0 -5],10);
% [indices,dists] = findNearestNeighbors(firstPtCloud,[0 0 -5],100);
% 
% newptCloudOut = select(firstPtCloud,indices);
% 
% % Posizione q_obstacle rispetto alla mappa 
% vicinissimo = newptCloudOut.Location(1,:);
% 
% % Distanza rispetto all'origine
% norm(vicinissimo);
% 
% % Distanza rispetto al drone nell'istante in cui è generata la nuvola di punti
% norm(vicinissimo-[0 0 -5]); 
% pcshow(newptCloudOut);

