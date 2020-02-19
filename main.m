% script to make coordinate tfs

% nedd to follow these steps in order to make it work: https://www.mathworks.com/matlabcentral/answers/329662-unable-to-access-rosbag-topics

%%
clear all; close all;

paths = ["bags/200212_154801_corredor_lab_reta.bag","bags/200212_160813_corredor_lab_ziguezague.bag","bags/200212_165047_corredor_lab_reta_180.bag",...
    "bags/200212_171052_corredor_lab_reta_ziguezague.bag"];

bag = rosbag(paths(4));

% we need 3 tfs
% 1 - world  -> ArUco      - measured
% 2 - ArUco  -> camera     - given by ar_track_alvar (but camera -> QR)
% 3 - camera -> robot      - static tf


%% tf #1 - world  -> ArUco
tf_world_arucos = initializeAruco();                        % Nx7
tf_world_arucos = quaternionToHomogeneous(tf_world_arucos); % 4x4xN


%% tf #2 - ArUco -> camera (or actually, camera -> ArUco)
bagSelect_arMsgs = select(bag,'Topic', '/ar_pose_marker');  
tf_aux = readMessages(bagSelect_arMsgs);          

tf_camera_arucos = zeros(size(tf_aux,1),7);
count = 1;
for i=1:size(tf_aux,1)
    %disp(i);
    if (size(tf_aux{i}.Markers,1) > 0 && tf_aux{i}.Markers(1).Id ~= 0) % checking if we detected any            
            % always getting the first marker
            tf_camera_arucos(i, 1) = tf_aux{i}.Markers(1).Pose.Pose.Position.X;
            tf_camera_arucos(i, 2) = tf_aux{i}.Markers(1).Pose.Pose.Position.Y;
            tf_camera_arucos(i, 3) = tf_aux{i}.Markers(1).Pose.Pose.Position.Z;

            tf_camera_arucos(i, 4) = tf_aux{i}.Markers(1).Pose.Pose.Orientation.X;
            tf_camera_arucos(i, 5) = tf_aux{i}.Markers(1).Pose.Pose.Orientation.Y;
            tf_camera_arucos(i, 6) = tf_aux{i}.Markers(1).Pose.Pose.Orientation.Z;
            tf_camera_arucos(i, 7) = tf_aux{i}.Markers(1).Pose.Pose.Orientation.W;

            % savind IDs for each identified tag
            % didnt initialize it previously with zeros(size(tf_aux,1),1)
            % because we dont know exacty how many we're using, since we have
            % empty messages (that didnt recognize any markers)
            id_seq(count,1) =  tf_aux{i}.Markers(1).Id;
            count = count +1;
            %disp(strcat("ID = ",num2str(tf_aux{i}.Markers(1).Id), " and ~= ",num2str(tf_aux{i}.Markers(1).Id ~= 0)));
    end
end

tf_camera_arucos = quaternionToHomogeneous(tf_camera_arucos); % 4x4xN


%% tf #3 - camera -> robot

camera_frame_id = erase(tf_aux{1}.Markers(1).Header.FrameId,'/');
%tf_aux = getTransform(bag, camera_frame_id, 'base_link');

load('tf_aux.mat'); % loading axis_front_optical_frame to base_link transform

tf_camera_base_link = zeros(1,7);
tf_camera_base_link(1, 1) = tf_aux.Transform.Translation.X;
tf_camera_base_link(1, 2) = tf_aux.Transform.Translation.Y;
tf_camera_base_link(1, 3) = tf_aux.Transform.Translation.Z;

tf_camera_base_link(1, 4) = tf_aux.Transform.Rotation.X;
tf_camera_base_link(1, 5) = tf_aux.Transform.Rotation.Y;
tf_camera_base_link(1, 6) = tf_aux.Transform.Rotation.Z;
tf_camera_base_link(1, 7) = tf_aux.Transform.Rotation.W;


tf_camera_base_link = quaternionToHomogeneous(tf_camera_base_link); % 4x4xN

%% transforms
%tf_world_base_link = zeros(4,4,size(tf_camera_arucos,3));

count = 0;
for i=1:size(id_seq,1) % iterate only on messages that we saw an aruco
    
    aruco_id = id_seq(i,1);
    if(aruco_id <= 12)
        count = count + 1;
        disp(count);
        tf_world_base_link(:,:,count) = tf_world_arucos(:,:,aruco_id)*inv(tf_camera_arucos(:,:,count))*tf_camera_base_link;
        
        if (sum(tf_world_base_link(:,:,count) == 0))
            disp(aruco_id);
            tf_world_arucos(:,:,aruco_id)
            inv(tf_camera_arucos(:,:,count))
            tf_camera_base_link
        end
        
    end
end

%% plot

for i=1:size(tf_world_base_link,3)
    disp(i);
    trplot(tf_world_base_link(:,:,i));
    hold on;
end
axis auto;





