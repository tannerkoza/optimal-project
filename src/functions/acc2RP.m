function [roll, pitch] = acc2RP(acc)
roll = atan2(acc(2),acc(3)); % Roll
pitch = atan2(-acc(1),sqrt(acc(2)^2 + acc(3)^2));
end