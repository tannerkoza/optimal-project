function [data, freq]=importAPDM(PathName,FileName,ID)
    %%imports data from APDM Sensor
    % example:
    % [FileName,PathName] = uigetfile('*.h5','HDF5'); 
    % ex: data = importAPDM(PathName,FileName,'SI-000577');
    
    %ID=Sensor ID
    %data(:,1:3)=accelerometer (m/s^2)
    %data(:,4:6)=gyro (rad/s)
    %data(:,7:9)=magnetometer (uT)
    %data(:,10:13)=quaternion (q1:scalar q2,q3,q4 vector)
    R = eye(3); 
    freq = double(h5readatt(strcat(PathName,FileName),strcat('/',ID),'SampleRate')); 
    data(:,1:3) = h5read(strcat(PathName,FileName),strcat('/',ID,'/Calibrated/Accelerometers'))';
    data(:,4:6) = h5read(strcat(PathName,FileName),strcat('/',ID,'/Calibrated/Gyroscopes'))';
    
    %import mag data if available
    mag_enabled = h5readatt(strcat(PathName,FileName),strcat('/',ID),'MagnetometersEnabled');
    if mag_enabled==0
       data(:,7:9) = nan(size(data,1),3); 
    else 
       data(:,7:9) = h5read(strcat(PathName,FileName),strcat('/',ID,'/Calibrated/Magnetometers'))';
    end
    data(:,10:13) = h5read(strcat(PathName,FileName),strcat('/',ID,'/Calibrated/Orientation'))';
    
    data(:,1:3) = data(:,1:3)*R;
    data(:,4:6) = data(:,4:6)*R; 
    data(:,7:9) = data(:,7:9)*R;

    %{
    %convert rotation matrix to quaternion
    q(1)=0.5.*sqrt(1+R(1,1)+R(2,2)+R(3,3));
    q(2)=(R(3,2)-R(2,3))./(4.*q(1));
    q(3)=(R(1,3)-R(3,1))./(4.*q(1));
    q(4)=(R(2,1)-R(1,2))./(4.*q(1));
    %}
end
%{
function [ qOut ] = quatMultiply(q1, q2)
qOut=[q1(:,1).*q2(:,1)-q1(:,2).*q2(:,2)-q1(:,3).*q2(:,3)-q1(:,4).*q2(:,4),... 
      q1(:,1).*q2(:,2)+q1(:,2).*q2(:,1)+q1(:,3).*q2(:,4)-q1(:,4).*q2(:,3),...
      q1(:,1).*q2(:,3)-q1(:,2).*q2(:,4)+q1(:,3).*q2(:,1)+q1(:,4).*q2(:,2),...
      q1(:,1).*q2(:,4)+q1(:,2).*q2(:,3)-q1(:,3).*q2(:,2)+q1(:,4).*q2(:,1)]; 

end
%}


