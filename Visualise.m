close all;
clear;
clc;

%% Activate/Deactivate Robots

Robot2Active = true;

%% Load necessary variables
load('Ben1_V2.mat');
iwaa_2_JointAngles1=double(iwaa_2_JointAngles);
clear MagCovariance iwaa_2_JointAngles;

load('Ben3_V2.mat');
iwaa_2_JointAngles2=double(iwaa_2_JointAngles);
clear MagCovariance iwaa_2_JointAngles;
iwaa_2_JointAngles = [iwaa_2_JointAngles1;iwaa_2_JointAngles2];


%% Constants

% Define unit vectors in local magnet frame (assuming the magnet's north pole points along the local Z-axis)
localMagnetDirection = [0; 0; 1];

%Magentic moment of the EPM (Magnitude)
mu_EPM = 970.1; %from some old code, where does it come from?


% Permeability of free space
mu0 = 4*pi*1e-7; 

% Grid
[x, y, z] = meshgrid(linspace(-1, 1, 30), linspace(-1, 1, 30), linspace(-0.4, 1, 25));

%Robot joint limits
jointLimits = [-170,170;
               -90,120; % Elbow Up
               -170,170;
               -120,120;
               -170,170;
               -120,120;
               -175,175];

%% Define Robot Parameters


if Robot2Active

    %Load robot 2
    robot2 = importrobot('urdf/kuka_iiwa_2.urdf','DataFormat','row');

    %Create initial guess
    initialguess2 = iwaa_2_JointAngles(1,2:8);

    %Create Inverse Kinematic solver
    IK2 = generalizedInverseKinematics('RigidBodyTree', robot2,'ConstraintInputs', {'position','aiming','joint','orientation'});

end

% Number of steps in the trajectory
numSteps = length(iwaa_2_JointAngles)-1;

%% Setup Animation

% Create a figure for the animation
hFig = figure;

% Set the figure to fullscreen
set(hFig, 'units', 'normalized', 'outerposition', [0 0 1 1]);

% Set up the axes for the 3D plot
ax = axes('Parent', hFig);
hold(ax, 'on');
axis(ax, 'equal');
axis(ax, [-0.2 0.2 -0.2 0.2 -0.2 0.2]);  %[-1 1 -1 1 -0.42 1]; [-0.2 0.2 -0.2 0.2 -0.2 0.2]
grid(ax, 'on');
xlabel(ax, 'X');
ylabel(ax, 'Y');
zlabel(ax, 'Z');
title(ax, 'Robot Trajectories and Magnetic Field Visualization');

% Set the view to isometric
view(ax, -0.1442, 22.8223);

%% Setup Loop for animation

% Loop through each step in the trajectory
for step = 1:numSteps

    disp(step);

    %Current robot pos
    Angles2 = initialguess2;

    %% Robot End Effector Magnetic Fields

    %initialise Bx, By, Bz
    Bx2 = x.*0; By2 = x.*0; Bz2 = x.*0; 
        
    %Robot 2
    if Robot2Active
            
        % Compute transformation matrix for robots from base to end effector
        transformMatrix2 = getTransform(robot2, Angles2, 'magnet_center_link', 'base_link');
    
        % Extract rotation matrices from transformation matrices
        R2 = transformMatrix2(1:3, 1:3);
    
        % Get x,y,z position Magnet of Robot 2
        Mag2_x = transformMatrix2(1,4);
        Mag2_y = transformMatrix2(2,4);
        Mag2_z = transformMatrix2(3,4);
    
        % Calculate magnetic moment vectors
        m2 = mu_EPM * R2 * localMagnetDirection;
    
        % Calculate field components for the second EPM (robot2)
        x2 = x - Mag2_x;
        y2 = y - Mag2_y;
        z2 = z - Mag2_z;
        r2 = sqrt(x2.^2 + y2.^2 + z2.^2);
        rx2 = x2./r2; ry2 = y2./r2; rz2 = z2./r2;
        
        Bx2 = mu0/(4*pi) * (3*(m2(1)*rx2 + m2(2)*ry2 + m2(3)*rz2).*rx2 - m2(1))./r2.^3;
        By2 = mu0/(4*pi) * (3*(m2(1)*rx2 + m2(2)*ry2 + m2(3)*rz2).*ry2 - m2(2))./r2.^3;
        Bz2 = mu0/(4*pi) * (3*(m2(1)*rx2 + m2(2)*ry2 + m2(3)*rz2).*rz2 - m2(3))./r2.^3;
    
        % Remove singularities
        threshold = 0.1;
        Bx2(r2<threshold) = NaN; By2(r2<threshold) = NaN; Bz2(r2<threshold) = NaN;
    end
    
    
    % Sum the magnetic fields from all dipoles
    Bx_total = Bx2;
    By_total = By2;
    Bz_total = Bz2;

    % Clear previous robots and fields for the new frame (optional)
    cla(ax);

    if Robot2Active
        show(robot2,Angles2,"Frames","off");
        hold on
        %addOrientationArrows(transformMatrix2);
        
    end
    hold on
    quiver3(x, y, z, Bx_total, By_total, Bz_total);
    axis([-1 1 -1 1 -0.42 1]);


    % Pause to control the speed of animation
    pause(0.1);

    % Capture frames for a video
    %frames(step) = getframe(hFig);

    % Update Robot position for next iteration
    initialguess2 = iwaa_2_JointAngles(step+1,2:8);
end

% Save the animation as a video file
%video = VideoWriter('robot_trajectory2.avi');
%video.FrameRate = 10;
%open(video);
%writeVideo(video, frames);
%close(video);
