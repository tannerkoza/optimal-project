function out = importIMUBag(pathName, fileName, imuTopic)
% DESCRIPTION: This function reads a specified rosbag file and saves the IMU topic's data in a structure.
% PARAMS:
%   - pathName: path of desired directory
%   - fileName: file within desired directory
%   - gpsTopic: ROS topic corresponding to gps data
% OUTPUT: 
%   - out: a structure of gps imu and time data
%   - out.gyro: 3-axis gyroscope data X, Y, and Z (rad/s)
%   - out.acc: 3-axis accelerometer data X, Y, and Z (m/s^2)
%   - out.quat: Hamilton quaternion (wxyz)
%   - out.time: time vector (s)

%% Import Bag

bag = rosbag(strcat(pathName, fileName)); 
bagSelect = select(bag, 'Topic', imuTopic); 
msg =  readMessages(bagSelect,'dataFormat','struct');

%% Parse GPS IMU BAG

% Preallocation
numMes = bagSelect.NumMessages; 

gyro = zeros(numMes,3);
acc = zeros(numMes,3);
quat = zeros(numMes,4); 
timeS = zeros(numMes,1);
timeNS = zeros(numMes,1); 

% Vector Population
 for i=1:numMes

     timeS(i) = msg{i}.Header.Stamp.Sec;
     timeNS(i) = msg{i}.Header.Stamp.Nsec;

     gyro(i,1) = msg{i}.AngularVelocity.X;
     gyro(i,2) = msg{i}.AngularVelocity.Y;
     gyro(i,3) = msg{i}.AngularVelocity.Z;

     acc(i,1) = msg{i}.LinearAcceleration.X;
     acc(i,2) = msg{i}.LinearAcceleration.Y;
     acc(i,3) = msg{i}.LinearAcceleration.Z;

     quat(i,1) = msg{i}.Orientation.W;
     quat(i,2) = msg{i}.Orientation.X;
     quat(i,3) = msg{i}.Orientation.Y;
     quat(i,4) = msg{i}.Orientation.Z;


 end

 % Create Time Vector
time = timeS + (timeNS.*1e-9);

% Populate Output Structure
out.gyro = gyro; 
out.acc = acc; 
out.quat = quat; 
out.time = time;

end

