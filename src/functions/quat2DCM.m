function [R] = quat2DCM(q)


% Dimension Handling
sz = size(q);
numQ = sz(2);

if sz(1) ~= 4
    q = q';
    [~, numQ] = size(q);
end

    R = zeros(3,3,numQ);
    
    for i=1:numQ
        R(1,1,i)= q(i,1)^2+q(i,2)^2-q(i,3)^2-q(i,4)^2;
        R(1,2,i)=-2*q(i,1)*q(i,4)+2*q(i,2)*q(i,3);
        R(1,3,i)= 2*q(i,1)*q(i,3)+2*q(i,2)*q(i,4);
        R(2,1,i)= 2*q(i,1)*q(i,4)+2*q(i,2)*q(i,3);
        R(2,2,i)= q(i,1)^2-q(i,2)^2+q(i,3)^2-q(i,4)^2;
        R(2,3,i)=-2*q(i,1)*q(i,2)+2*q(i,3)*q(i,4);
        R(3,1,i)=-2*q(i,1)*q(i,3)+2*q(i,2)*q(i,4);
        R(3,2,i)=2*q(i,1)*q(i,2)+2*q(i,3)*q(i,4);
        R(3,3,i)=q(i,1)^2-q(i,2)^2-q(i,3)^2+q(i,4)^2;
    end

end
