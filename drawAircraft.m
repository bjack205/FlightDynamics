
function drawAircraft(uu)

N = (length(uu)-1)/12;

% process inputs to function
for i = 1:N
    k = 12*(i-1);
    pn(i)       = uu(1+k);       % inertial North position
    pe(i)       = uu(2+k);       % inertial East position
    pd(i)       = uu(3+k);
    u(i)        = uu(4+k);
    v(i)        = uu(5+k);
    w(i)        = uu(6+k);
    phi(i)      = uu(7+k);       % roll angle
    theta(i)    = uu(8+k);       % pitch angle
    psi(i)      = uu(9+k);       % yaw angle
    p(i)        = uu(10+k);       % roll rate
    q(i)        = uu(11+k);       % pitch rate
    r(i)        = uu(12+k);       % yaw rate
end
t               = uu(end);       % time



% define persistent variables
persistent vehicle_handle;
persistent Vertices
persistent Faces
persistent facecolors
persistent window scaling
persistent init

if nargin == 1
    multiplane = false;
    id = 1;
else
    multiplane = true;
end

% first time function is called, initialize plot and persistent vars
if t==0
    figure(1)
    init = true;
    
    if init
        clf
        ground = 1e9;
        alt = 0;
        
        patch([ground, ground,-ground,-ground],[ground,-ground,-ground,ground],-[alt alt alt alt],[0 0.5 0])
        title('Vehicle')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the view angle for figure
        axis equal
        light('Position',[-0.4 0.2 0.9],'Style','infinite');
        lighting gouraud
        grid on
        hold on
        init = false;
    end
    
    scaling = 10;
    window = 100;
    for id = 1:N
        [Vertices,Faces,facecolors] = defineVehicleBody(scaling);
        vehicle_handle{id} = drawVehicleBody(Vertices,Faces,facecolors,...
            pn(id),pe(id),pd(id),phi(id),theta(id),psi(id),...
            []);
    end
    FollowPlane(vehicle_handle{1},pn(1),pe(1),pd(1),window)
    
else
    FollowPlane(vehicle_handle{1},pn(1),pe(1),pd(1),window)
    for id = 1:N
        drawVehicleBody(Vertices,Faces,facecolors,...
            pn(id),pe(id),pd(id),phi(id),theta(id),psi(id),...
            vehicle_handle{id});
    end
    
end

if multiplane
    Rbv = quat2rmat(euler2quat(phi,theta,psi));
    R = [...
        0, 1, 0;...
        1, 0, 0;...
        0, 0, -1;...
        ];
    pos = [pn;pe;pd];
    offset = Rbv'*[1;0;0]*scaling;
    pos = pos-offset;
    xyz = R*pos;
    if t == 0
        plane_label{id} = text(xyz(1),xyz(2),xyz(3),['Plane ' num2str(id)]);
    else
        plane_label.Position = xyz;
    end
    
end
drawnow
end


%=======================================================================
% drawVehicle
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawVehicleBody(V,F,patchcolors,...
    pn,pe,pd,phi,theta,psi,...
    handle)
V = rotate(V, phi, theta, psi);  % rotate vehicle
V = translate(V, pn, pe, pd);  % translate vehicle

% transform vertices from NED to XYZ (for matlab rendering)
R = [...
    0, 1, 0;...
    1, 0, 0;...
    0, 0, -1;...
    ];
V = R*V;

if isempty(handle)
    handle = patch('Vertices', V', 'Faces', F,...
        'FaceVertexCData',patchcolors,...
        'FaceColor','flat');
    
else
    set(handle,'Vertices',V','Faces',F);
end

end

function FollowPlane(vehicle_handle,pn,pe,pd,window)
if pd > 0
    bottom = pd+window;
else
    bottom = 0;
end
xlim(vehicle_handle.Parent,pe+[-window window])
ylim(vehicle_handle.Parent,pn+[-window window])
zlim(vehicle_handle.Parent,[-bottom -pd+window])
end

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

% define rotation matrix (right handed)
R_roll = [...
    1, 0, 0;...
    0, cos(phi), sin(phi);...
    0, -sin(phi), cos(phi)];
R_pitch = [...
    cos(theta), 0, -sin(theta);...
    0, 1, 0;...
    sin(theta), 0, cos(theta)];
R_yaw = [...
    cos(psi), sin(psi), 0;...
    -sin(psi), cos(psi), 0;...
    0, 0, 1];
R = R_roll*R_pitch*R_yaw;
% note that R above either leaves the vector alone or rotates
% a vector in a left handed rotation.  We want to rotate all
% points in a right handed rotation, so we must transpose
R = R';

% rotate vertices
pts = R*pts;

end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

pts = pts + repmat([pn;pe;pd],1,size(pts,2));

end

% end translate


%=======================================================================
% defineVehicleBody
%=======================================================================
function [V,F,facecolors] = defineVehicleBody(scaling)

% Parameters
fuse_l1 = 1;
fuse_l2 = fuse_l1/2;
fuse_l3 = 4;
fuse_h = 0.5;
fuse_w = 0.5;
wing_l = 1;
wing_w = 5;
tailwing_w = 2.5;
tailwing_l = .75;
tail_h = 1.5;

% Define the vertices (physical location of vertices
V = [...
    fuse_l1, 0, 0;...   % pt 1
    fuse_l2, fuse_w/2, -fuse_h/2;... % pt 2
    fuse_l2, -fuse_w/2, -fuse_h/2;...   % pt 3
    fuse_l2, -fuse_w/2, fuse_h/2;...  % pt 4
    fuse_l2, fuse_w/2, fuse_h/2;...  % pt 5
    -fuse_l3, 0, 0;... % pt 6
    0, wing_w/2, 0; % pt 7
    -wing_l, wing_w/2, 0; % pt 8
    -wing_l, -wing_w/2, 0; % pt 9
    0, -wing_w/2, 0; % pt 10
    -fuse_l3+tailwing_l, tailwing_w/2, 0; % pt 11
    -fuse_l3, tailwing_w/2, 0; % pt 12
    -fuse_l3, -tailwing_w/2, 0; % pt 13
    -fuse_l3+tailwing_l, -tailwing_w/2, 0; % pt 14
    -fuse_l3+tailwing_l, 0, 0;
    -fuse_l3, 0, -tail_h;
    ]';

% define faces as a list of vertices numbered above
F = [...
    1, 2, 3, NaN;...  % nose top
    1, 3, 4, NaN;...  % nose left
    1, 4, 5, NaN;...  % nose bottom
    1, 2, 5, NaN;...  % nose right
    2, 3, 6, NaN;...  % fuse top
    3, 4, 6, NaN;...  % fuse left
    5, 4, 6, NaN;...  % fuse bottom
    5, 2, 6, NaN;...  % fuse right
    15, 16, 6, NaN;... % tail
    10, 7, 8, 9;... % wing
    14, 11, 12, 13; ... % tail wing
    ];

% define colors for each face
myred = [1, 0, 0];
mygreen = [0.4667    0.6745    0.1882];
mytan = [0.7490    0.7490         0];
myblue = [0, 0, 1];
myyellow = [1, 1, 0];
mycyan = [0, 1, 1];

facecolors = [...
    mygreen;...    % nose
    mygreen;...    % nose
    mygreen;...    % nose
    mygreen;...    % nose
    mygreen;...    % fuse
    mygreen;...    % fuse
    mygreen;...    % fuse
    mygreen;...    % fuse
    mytan;...     % tail
    mytan;...    % wing
    mytan;...    % wing
    ];
V = V*scaling;
end
