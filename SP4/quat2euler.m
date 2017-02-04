function [phi,theta,psi] = quat2euler(q)
phi = atan2(2*(q(1)*q(2) + q(3)*q(4)), q(1)^2 + q(4)^2 - q(2)^2 - q(3)^2 );
theta = asin(2*(q(1)*q(3) - q(2)*q(4)));
psi = atan2(2*(q(1)*q(4) + q(2)*q(3)), q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2 );