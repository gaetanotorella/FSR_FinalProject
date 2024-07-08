%% TRIGGER
video = 0;

%% SCENARIO
% Initialize the scenario 
scene = uavScenario(UpdateRate=1/Ts,ReferenceLocation=[0 0 0]);
scene.addInertialFrame("NED","MAP",trvec2tform([0 0 0])); 

% Create a ground
addMesh(scene,"polygon",{[-5 -5; 10 -5; 10 10; -5 10] [-0.5 0]},[0.3 0.3 0.3]);

% Add obstacles meshes
addMesh(scene,"cylinder",{[obs_1(1:3)],[obs_1(4:5)]},[1 0 0]);
addMesh(scene,"cylinder",{[obs_2(1:3)],[obs_2(4:5)]},[1 0 0]);

% (scenario, "oggetto", {[x y radius], [z altezza], [colore]}


%% UAV 
% Platform/UAV initial position and orientation
initpos = q_0'; 
initori = [0 0 0];
initvel = [0 0 0];
initacc = [0 0 0];

% Add UAV Platform to the scenario and scale it for easier visualization
platform = uavPlatform("platformUAV",scene,ReferenceFrame="NED",...
    InitialPosition=initpos,InitialOrientation=eul2quat(initori));

updateMesh(platform,"quadrotor",{1},[0 0 0],eul2tform([0 0 pi]));


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
