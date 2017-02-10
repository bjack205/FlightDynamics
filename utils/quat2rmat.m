function rmat = quat2rmat(q)
%QUAT2RMAT Converts quaternions to rotation matrices
%   INPUTS:
%       q is an Nx4 matrix of quaternions
%
%   OUTPUTS:
%       rmat is a 3x3xN matrix of rotation matrices
%
% Brian Jackson August 2016
% Brigham Young University

%Normalize Quaternion
q = quatnorm(q);

rmat = [...
    1-2*q(:,3).^2-2*q(:,4).^2           2*q(:,2).*q(:,3)-2*q(:,4).*q(:,1)   2*q(:,2).*q(:,4)+2*q(:,3).*q(:,1);...
    2*q(:,2).*q(:,3)+2*q(:,4).*q(:,1)   1-2*q(:,2).^2-2*q(:,4).^2           2*q(:,3).*q(:,4)-2*q(:,2).*q(:,1);...
    2*q(:,2).*q(:,4)-2*q(:,3).*q(:,1)   2*q(:,3).*q(:,4)+2*q(:,2).*q(:,1)   1-2*q(:,2).^2-2*q(:,3).^2];

rmat = permute(reshape(rmat',3,size(q,1),3),[3 1 2]);

end