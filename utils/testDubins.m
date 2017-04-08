function testDubins
start_node= [0,0,10,-90*pi/180];
end_node = [300,300,10,0*pi/180];

dubinsPath = dubinsParameters(start_node, end_node, 75);
if ~isempty(dubinsPath)
    [X,Y,Z] = pointsAlongPath(dubinsPath,0.1);

    figure(5); cla; hold on
    scale = 100;
    LineWidth = 2;
    quiver(start_node(2),start_node(1),sin(start_node(4)),cos(start_node(4)),scale,'LineWidth',LineWidth)
    quiver(end_node(2),end_node(1),sin(end_node(4)),cos(end_node(4)),scale,'LineWidth',LineWidth)
    scatter(Y,X,'.')
end
end

function [X,Y,Z] = pointsAlongPath(dubinsPath, Del)
vec_angle = @(s,e) atan2(e(2)-s(2),e(1)-s(1));

% First circle
theta1 = vec_angle(dubinsPath.cs,dubinsPath.ps);
theta2 = vec_angle(dubinsPath.cs,dubinsPath.w1);
[X1,Y1] = circlepoints(dubinsPath.cs,dubinsPath.R,theta1,theta2,dubinsPath.lams);
Z1 = ones(size(X1))*dubinsPath.ps(3);

% Line
start_node = dubinsPath.w1;
X2 = start_node(1);
Y2 = start_node(2);
Z2 = start_node(3);

end_node = dubinsPath.w2;
q = end_node(1:3)-start_node(1:3);
L = norm(q);
q = q/L;

w = start_node(1:3);
for i=2:floor(L/Del)
    w = w + Del*q;
    X2 = [X2; w(1)];
    Y2 = [Y2; w(2)];
    Z2 = [Z2; w(3)];
end

% End Circle
theta1 = vec_angle(dubinsPath.ce,dubinsPath.w2);
theta2 = vec_angle(dubinsPath.ce,dubinsPath.w3);
[X3,Y3] = circlepoints(dubinsPath.ce,dubinsPath.R,theta1,theta2,dubinsPath.lame);
Z3 = ones(size(X3))*dubinsPath.ps(3);

% Combine Points
X = [X1;X2;X3];
Y = [Y1;Y2;Y3];
Z = [Z1;Z2;Z3];


end

function [X,Y] = circlepoints(c,R,theta1,theta2,lamda)
angle = 2*pi*(lamda==-1)+lamda*mod(theta2-theta1,2*pi);

dtheta = 1*pi/180;
N = angle/dtheta;

angles = theta1+lamda*linspace(0,angle,N)';
X = cos(angles)*R+c(1);
Y = sin(angles)*R+c(2);
end