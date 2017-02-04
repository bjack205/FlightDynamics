function eulerstate = euler2quat_state(x)
e = euler2quat(x(7),x(8),x(9));
eulerstate = [x(1:6);e';x(10:12)];