function x2 = PlaneOffset(x,offset)
x2 = x;
x = euler2quat_state(x);
R = quat2rmat(x(7:10)');
x2(1:3) = R*offset+x(1:3);