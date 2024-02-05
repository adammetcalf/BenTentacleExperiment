close all;
clear;
clc;

%% Open rosbag, create object in MATLAB
bag = rosbag('Ben1_2024-02-01-16-44-32.bag');

%% Display info to user
bagInfo = rosbag('info','Ben1_2024-02-01-16-44-32.bag');

%% Read the HE messages
HE_data = select(bag,'Topic','/he_data_topic');
HE_messages = readMessages(HE_data,'DataFormat','struct');

% Initialize arrays with NaNs for timestamps and data
MagFieldX = nan(numel(HE_messages), 1);
MagFieldY = nan(numel(HE_messages), 1);
MagFieldZ = nan(numel(HE_messages), 1);
MagCovariance = nan(numel(HE_messages), 9); % Assuming each message has a 9-element covariance
HE_Timestamps = nan(numel(HE_messages), 1);

% Loop through each cell, extract the Data and Timestamp
for n = 1:numel(HE_messages)
    % Convert ROS time (Sec + Nsec) to seconds as double
    HE_Timestamps(n) = double(HE_messages{n}.Header.Stamp.Sec) + double(HE_messages{n}.Header.Stamp.Nsec) * 1e-9;
    MagFieldX(n) = HE_messages{n}.MagneticField_.X;
    MagFieldY(n) = HE_messages{n}.MagneticField_.Y;
    MagFieldZ(n) = HE_messages{n}.MagneticField_.Z;
    MagCovariance(n, :) = HE_messages{n}.MagneticFieldCovariance;
end

% Prepend timestamps to the data arrays
MagFieldX = [HE_Timestamps, MagFieldX];
MagFieldY = [HE_Timestamps, MagFieldY];
MagFieldZ = [HE_Timestamps, MagFieldZ];
% Assuming MagCovariance is a separate structure you're handling differently

%% Read the Joint messages
iwaa_2_Joints = select(bag, 'Topic', '/joint2/repub');
iwaa_2_messages = readMessages(iwaa_2_Joints, 'DataFormat', 'struct');

%% Filter MessageList for '/joint2/repub' topic to extract timestamps
iwaa_2_MsgList = bag.MessageList(strcmp(bag.MessageList.Topic, "/joint2/repub"), :);


%% Loop through each message to extract data and associate with timestamp
for n = 1:numel(iwaa_2_messages)
    iwaa_2_JointAngles(n, 1) = HE_Timestamps(n); % Assign timestamp (borrowed from HE sensors)
    iwaa_2_JointAngles(n, :) = iwaa_2_messages{n}.Data; 
end
