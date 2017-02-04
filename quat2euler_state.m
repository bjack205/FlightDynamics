function quatstate = quat2euler_state(x)
[phi,theta,psi] = quat2euler(x(7:10));
quatstate = [x(1:6);phi;theta;psi;x(11:13)];