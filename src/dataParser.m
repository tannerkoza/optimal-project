% Optimal Estimation Project - Data Parser

clear
clc

%% Restructure Data

% Directory Declaration
inDir = 'data/rawData/';
inFiles = [dir(strcat(inDir,'*.txt')); dir(strcat(inDir,'*.bag'))];
outDir = 'data/structData';

% Incorrect Working Directory Handling
numFiles = length(inFiles);

if numFiles == 0
    error('No files have been read. Run this file from the root directory.')
end

% IMU Topic
imuTopic = 'vectornav/IMU';
magTopic = 'vectornav/Mag';

% Structure Population
for i = 1:numFiles

    % Create Destination File
    thisFile = inFiles(i).name;
    [~, ~, fileExt] = fileparts(thisFile);
    outFile = strcat(strrep(thisFile,fileExt,''),'.mat');
    destFile = fullfile(outDir,outFile);

    if strcmp(fileExt,'.txt') == true

        % Import Data
        imuTable = readtable(thisFile);
        imuArr = table2array(imuTable);

        % Populate Struct
        imu.acc = imuArr(:,1:3);
        imu.gyro = imuArr(:,4:6);
        imu.mag = imuArr(:,7:9);

        save(destFile,"imu")
        clearvars imu

    else

        % Populate Struct
        imu = importIMUBag(inDir,thisFile,imuTopic);
        mag = importMagBag(inDir,thisFile,magTopic);
        save(destFile,'imu','mag')
        clearvars imu

    end

end