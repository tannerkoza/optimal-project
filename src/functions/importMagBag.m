function out = importMagBag(pathName, fileName, magTopic)
% DESCRIPTION: This function reads a specified rosbag file and saves the IMU topic's data in a structure.
% PARAMS:
%   - pathName: path of desired directory
%   - fileName: file within desired directory
%   - gpsTopic: ROS topic corresponding to gps data
% OUTPUT: 
%   - out: a structure of mag and time data
%   - out.mag: 3-axis gyroscope data X, Y, and Z (rad/s)
%   - out.time: time vector (s)

%% Import Bag

bag = rosbag(strcat(pathName, fileName)); 
bagSelect = select(bag, 'Topic', magTopic); 
msg =  readMessages(bagSelect,'dataFormat','struct');

%% Parse GPS IMU BAG

% Preallocation
numMes = bagSelect.NumMessages; 

mag = zeros(numMes,3);
timeS = zeros(numMes,1);
timeNS = zeros(numMes,1); 

% Vector Population
 for i=1:numMes

     timeS(i) = msg{i}.Header.Stamp.Sec;
     timeNS(i) = msg{i}.Header.Stamp.Nsec;

     mag(i,1) = msg{i}.MagneticField_.X;
     mag(i,2) = msg{i}.MagneticField_.Y;
     mag(i,3) = msg{i}.MagneticField_.Z;
     
 end

 % Create Time Vector
time = timeS + (timeNS.*1e-9);

% Populate Output Structure
out.mag = mag; 
out.time = time;

end