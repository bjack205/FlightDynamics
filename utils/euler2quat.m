function e = euler2quat(phi,theta,psi)
if nargin == 1
    psi = phi(3);
    theta = phi(2);
    phi = phi(1);
end
e(1) = cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
e(2) = cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
e(3) = cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
e(4) = sin(psi/2)*cos(theta/2)*cos(phi/2) - cos(psi/2)*sin(theta/2)*sin(phi/2);