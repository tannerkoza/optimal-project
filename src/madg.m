% clear
clc

load('vn300LD')
fs = 20;
dt = 1/fs;

q = zeros(length(imu.gyro)-1, 4); % preallocate
AHRS = MadgwickAHRS('SamplePeriod', dt, 'Beta', 0.3); % Madgwick AHRS object initialization

gyro = imu.gyro;
acc = imu.acc;
mag = mag.mag;

for i = 1:length(imu.gyro)-1
    
    AHRS.Update(gyro(i,:) * (pi/180), acc(i,:), mag(i,:));
    q(i, :) = AHRS.Quaternion;
    
end

ekf_pose = quaternion(q);  % convert quaternions to MATLAB quaternion object

o = quat2eul(ekf_pose);

plot(wrapToPi(o))