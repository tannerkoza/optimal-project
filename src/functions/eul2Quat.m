function q = eul2Quat(roll,pitch,yaw)
q = [cos(yaw/2)*cos(roll/2)*cos(pitch/2) + sin(yaw/2)*sin(roll/2)*sin(pitch/2);
    cos(yaw/2)*cos(roll/2)*sin(pitch/2) - sin(yaw/2)*sin(roll/2)*cos(pitch/2);
    cos(yaw/2)*sin(roll/2)*cos(pitch/2) + sin(yaw/2)*cos(roll/2)*sin(pitch/2);
    sin(yaw/2)*cos(roll/2)*cos(pitch/2) - sin(yaw/2)*sin(roll/2)*cos(pitch/2)];
end