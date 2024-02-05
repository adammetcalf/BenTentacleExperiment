close all;
clear;
clc;

%% File name for saving mat file
filename = 'Ben3_V3.mat';

%% Open rosbag, create object in MATLAB
bag = rosbag('Ben3_2024-02-01-16-59-00.bag');

%% Display info to user
bagInfo = rosbag('info','Ben3_2024-02-01-16-59-00.bag');

%% Read the HE messages
HE_data = select(bag,'Topic','/he_data_topic');
HE_messages = readMessages(HE_data,'DataFormat','struct');

% Loop through each cell and extract the Data
for n = 1:numel(HE_messages)
    MagFieldX(n, :) = HE_messages{n}.MagneticField_.X;
    MagFieldY(n, :) = HE_messages{n}.MagneticField_.Y;
    MagFieldZ(n, :) = HE_messages{n}.MagneticField_.Z;
    MagCovariance(n, :) = HE_messages{n}.MagneticFieldCovariance;
end

% Build Magfield
MagField = [MagFieldX,MagFieldY,MagFieldZ];

%% Read the Joint messages
iwaa_2_Joints = select(bag,'Topic','/joint2/repub');
iwaa_2_messages = readMessages(iwaa_2_Joints,'DataFormat','struct');

% Loop through each cell and extract the Data
for n = 1:numel(iwaa_2_messages)
    iwaa_2_JointAngles(n, :) = iwaa_2_messages{n}.Data;
end

% Loop through each cell and extract the Data
for n = 1:numel(HE_messages)
    MagFieldX(n, :) = HE_messages{n}.MagneticField_.X;
    MagFieldY(n, :) = HE_messages{n}.MagneticField_.Y;
    MagFieldZ(n, :) = HE_messages{n}.MagneticField_.Y;
    MagCovariance(n, :) = HE_messages{n}.MagneticFieldCovariance;
end

%% Ensure correct array lengths (iwaa_2_JointAngles is master)

if length(iwaa_2_JointAngles) == length(MagField)
    % Do nothing
elseif length(iwaa_2_JointAngles) < length(MagField)
    % Remove extra entries from the end of MagField
    MagField = MagField(1:length(iwaa_2_JointAngles), :);
    MagCovariance = MagCovariance(1:length(iwaa_2_JointAngles), :);
else
    % Add extra entries into MagField by duplicating the final entry
    diff = length(iwaa_2_JointAngles) - length(MagField);
    for i = 1:diff
        MagField = [MagField; MagField(end, :)];
        MagCovariance = [MagCovariance;MagCovariance(end, :)];
    end
end

%% Add time and finalise arrays

Time(1) = 0;
for i=1:length(iwaa_2_JointAngles)-1
    Time(i+1) = Time(i) + 0.01;
end

iwaa_2_JointAngles = [Time',iwaa_2_JointAngles];
MagField = [Time', MagField];
MagCovariance = [Time',MagCovariance];

save(filename, "iwaa_2_JointAngles", "MagField", "MagCovariance");



