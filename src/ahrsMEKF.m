%% AHRS - Multiplicative Extended Kalman Filter 
% The following is an implementation of an Attitude and Heading Reference
% System using data recorded from a 9DOF IMU (MARG) fused using a Multiplicative
% EKF. The output of the EKF is then compared to the Madgwick AHRS
% algorithm.

% clear
% close all
clc

folder = fileparts(which('ahrs_mekf.m')); % add all subfolders at run
addpath(genpath(folder));

%% IMU Data Import

load('vn300LD.mat');

accel_data = imu.acc;   % sepearate individual sensor data 
gyro_data = imu.gyro;
mag_data = mag.mag;
time = imu.time;

%% Parameter & Storage Variable Initialization

g = [0;0;-9.81]; % gravity (m/s^2)
gyrNoise = 0.001;
accNoise = 0.009;
magNoise = accNoise;
measNoise = [accNoise accNoise accNoise magNoise magNoise magNoise].^2;

ekf_orien = zeros(4, length(gyro_data)); % preallocate quaternion attitude

%% Multiplicative Kalman Filter
% Initialization

q = [1; zeros(3,1)]; % initial quaternion attitude

Q = gyrNoise^2 * eye(3); % process noise covariance matrix
R = diag(measNoise); % measurement noise covariance matrix

C = ((q(1)^2-q(2:4)'*q(2:4)).*eye(3) + 2*q(2:4)*q(2:4)' + 2*q(1)*skew(q(2:4))); % rotation matrix from body to inertial frame

C(1, 1) = q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2;
C(1, 2) = 2 * (q(2) * q(3) - q(1) * q(4));
C(1, 3) = 2 * (q(1) * q(3) + q(2) * q(4));

C(2, 1) = 2 * (q(1) * q(4) + q(2) * q(3));
C(2, 2) = q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2;
C(2, 3) = 2 * (q(3) * q(4) - 2*(q(1) * q(2))); 

C(3, 1) = 2 * (q(2) * q(4) - q(1) * q(3));
C(3, 2) = 2 * (q(1) * q(2) + q(3) * q(4));
C(3, 3) = q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2;

W = C; % initial jacobian of state estimate (A*x) wrt noise
P = W*Q*W'; % initialized state estimate covariance

for i = 2:length(gyro_data)-1

    dt = time(i)-time(i-1);

% Quaternion Attitude Prediction
    
    A(1,1) = 1; % state transition matrix 
    A(1,2) = -0.5*gyro_data(i,1)*dt;
    A(1,3) = -0.5*gyro_data(i,2)*dt;
    A(1,4) = -0.5*gyro_data(i,3)*dt;
    
    A(2,1) = 0.5*gyro_data(i,1)*dt;
    A(2,2) = 1;
    A(2,3) = 0.5*gyro_data(i,3)*dt;
    A(2,4) = -0.5*gyro_data(i,2)*dt;
    
    A(3,1) = 0.5*gyro_data(i,2)*dt;
    A(3,2) = -0.5*gyro_data(i,3)*dt;
    A(3,3) = 1;
    A(3,4) = 0.5*gyro_data(i,1)*dt;
    
    A(4,1) = 0.5*gyro_data(i,3)*dt;
    A(4,2) = 0.5*gyro_data(i,2)*dt;
    A(4,3) = -0.5*gyro_data(i,1)*dt;
    A(4,4) = 1;

    q = A*q; % quaternion attitude prediction (propagation)

% Error State Time Update (Prediction Step)

%     x = zeros(3,1); % error state vector (eta)
    P = P + W*Q*W';

% Error State Innovation Measurement

    C(1, 1) = q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2;
    C(1, 2) = 2 * (q(2) * q(3) - q(1) * q(4));
    C(1, 3) = 2 * (q(1) * q(3) + q(2) * q(4));
    
    C(2, 1) = 2 * (q(1) * q(4) + q(2) * q(3));
    C(2, 2) = q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2;
    C(2, 3) = 2 * (q(3) * q(4) - 2*(q(1) * q(2))); 
    
    C(3, 1) = 2 * (q(2) * q(4) - q(1) * q(3));
    C(3, 2) = 2 * (q(1) * q(2) + q(3) * q(4));
    C(3, 3) = q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2; % rotation matrix from body to inertial frame

    W = dt*eye(3)*C;
    
    h_accel = C'*g; % predicted accelerometer measurements

    inertial_mag =  C*mag_data(i,:)'; % rotate body mag vector to inertial frame
    inertial_mag_ref = [norm([inertial_mag(1) inertial_mag(2)]); 0; inertial_mag(3)]; % normalize at current measurement's inclination
    h_mag = C'*inertial_mag_ref; % predicted magnetometer measurements

    h = [h_accel; h_mag]; % combine predicted measurements

    y = ([accel_data(i,:)'; mag_data(i,:)'] - h); % innovation/residual measurement

    H_accel = C'*skew(g);
    H_mag = C'*skew(inertial_mag_ref);

    H = [H_accel; H_mag];   % measurement transition matrix

% Error State Measurement Update (Correction Step)

    S = H*P*H' + R; % covariance of the measurement residual
    
    K = P*H'*(S)^-1; % near-optimal Kalman Gain

    x = K*y; % update error state

    nom = quaternion(q');
    error = quaternion([1; 0.5*x]');
 
    q = compact(error*nom)';
    q = q/norm(q);

    P = P - K*S*K'; % updated covariance of state estimate

    ekf_orien(:,i) = q;

end

%% Full State EKF

% fsekf_eul = fsekf(freq,gyro_data,accel_data,mag_data);

%% Plot Orientation

ekf_pose = quaternion(ekf_orien');  % convert quaternions to MATLAB quaternion object

ekf_eul = quat2eul(ekf_pose); % convert MATLAB quaternions to Euler angles

eulVN = quat2eul(imu.quat);

figure
movegui('northwest')
plot(rad2deg(ekf_eul(:,1)))
hold on
plot(rad2deg(eulVN(:,1)),'.')
legend('MEKF Yaw', 'FSEKF Yaw')
xlabel('Time (s)')
ylabel('Rotation Angle (rads)')
title('Filtered Yaw Comparison')

figure
movegui('north')
plot(rad2deg(ekf_eul(:,2)))
hold on
plot(rad2deg(eulVN(:,2)),'.')
legend('MEKF Pitch', 'FSEKF Pitch')
xlabel('Time (s)')
ylabel('Rotation Angle (rads)')
title('Filtered Pitch Comparison')

figure
movegui('northeast')
plot(rad2deg(ekf_eul(:,3)))
hold on
plot(rad2deg(eulVN(:,3)),'.')
legend('MEKF Roll', 'FSEKF Roll')
xlabel('Time (s)')
ylabel('Rotation Angle (rads)')
title('Filtered Roll Comparison')