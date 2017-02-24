function y = autopilot(uu,P)
%
% autopilot for mavsim
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   9/30/2014 - RWB
%   
    
    % process inputs
    NN = 0;
%    pn       = uu(1+NN);  % inertial North position
%    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
%    alpha    = uu(5+NN);  % angle of attack
%    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
%    Vg       = uu(13+NN); % ground speed
%    wn       = uu(14+NN); % wind North
%    we       = uu(15+NN); % wind East
%    psi      = uu(16+NN); % heading
%    bx       = uu(17+NN); % x-gyro bias
%    by       = uu(18+NN); % y-gyro bias
%    bz       = uu(19+NN); % z-gyro bias
    NN = NN+19;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
    NN = NN+3;
    t        = uu(1+NN);   % time
    
    autopilot_version = 2;
        % autopilot_version == 1 <- used for tuning
        % autopilot_version == 2 <- standard autopilot defined in book
        % autopilot_version == 3 <- Total Energy Control for longitudinal AP
    switch autopilot_version
        case 1,
           [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 2,
           [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 3,
               [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
    end
    y = [delta; x_command];
    
end
    
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot versions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_tuning
%   - used to tune each loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    mode = 5;
    switch mode
        case 1, % tune the roll loop
            phi_c = chi_c; % interpret chi_c to autopilot as course command
            if t == 0
                delta_a = roll_hold(phi_c, phi, p, 1, P);
            else
                delta_a = roll_hold(phi_c, phi, p, 0, P);
            end
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 2, % tune the course loop
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                delta_a = roll_hold(phi_c, phi, p, 1, P);
            else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                delta_a = roll_hold(phi_c, phi, p, 0, P);
            end                
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 3, % tune the throttle to airspeed loop and pitch loop simultaneously
            theta_c = 20*pi/180 + h_c;
            chi_c = 0;
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 4, % tune the pitch to airspeed loop 
            chi_c = 0;
            delta_t = P.u_trim(4);
            if t==0,
                init = true;
            else
                init = false;
            end
            phi_c   = course_hold(chi_c, chi, r, init, P);
            delta_a = roll_hold(phi_c, phi, p, init, P);
            
            theta_c = airspeed_with_pitch_hold(Va_c, Va, init, P);
            delta_e = pitch_hold(theta_c, theta, q, init, P);
            
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 5, % tune the pitch to altitude loop 
            chi_c = 0;
            if t==0,
                init = true;
            else
                init = false;
            end
            phi_c   = course_hold(chi_c, chi, r, init, P);
            delta_a = roll_hold(phi_c, phi, p, init, P);
            
            delta_t = airspeed_with_throttle_hold(Va_c, Va, init, P);
            %delta_t = P.u_trim(4);
            theta_c = altitude_hold(h_c, h, init, P);
            delta_e = pitch_hold(theta_c, theta, q, init, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 6 % Tune pitch loop
            theta_c = h_c; 
            chi_c = 0;
            phi_c = 0;
            if t==0
                delta_e = pitch_hold(theta_c, theta, q, 1, P);
            else
                delta_e = pitch_hold(theta_c, theta, q, 0, P);
            end
            delta_a = P.u_trim(2);
            delta_t = P.u_trim(4);
            delta_r = P.u_trim(3);
        case 7 % Tune Altitude hold
            chi_c = 0;
            phi_c = 0;
            if t==0
                theta_c = altitude_hold(h_c, h, 1, P);
                delta_e = pitch_hold(theta_c, theta, q, 1, P);
            else
                theta_c = altitude_hold(h_c, h, 0, P);
                delta_e = pitch_hold(theta_c, theta, q, 0, P);
            end
            delta_a = P.u_trim(2);
            delta_t = P.u_trim(4);
            delta_r = P.u_trim(3);
        case 8 % Speed with throttle
            chi_c = 0;
            if t==0,
                init = true;
            else
                init = false;
            end
            phi_c   = course_hold(chi_c, chi, r, init, P);
            delta_a = roll_hold(phi_c, phi, p, init, P);
            
            theta_c = 0;
            delta_t = airspeed_with_throttle_hold(Va_c, Va, init, P);
            delta_e = P.u_trim(1);
            delta_r = P.u_trim(3);
            
            
    end
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_uavbook
%   - autopilot defined in the uavbook
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0
        init_lat = true;
    else
        init_lat = false;
    end
    % assume no rudder, therefore set delta_r=0
    delta_r = 0;%coordinated_turn_hold(beta, 1, P);
    phi_c   = course_hold(chi_c, chi, r, init_lat, P);
    delta_a = roll_hold(phi_c, phi, p, init_lat, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot
    
    % define persistent variable for state of altitude state machine
    persistent altitude_state;
    persistent init;
    persistent state_names
    % initialize persistent variable
    if t==0,
        init = true;
        h = -P.pd0;
        Va = P.Va0;
        state_names = {'Take_off';'Climb Zone';'Descend Zone';'Altitude Hold Zone'};
    end
    
    prev_state = altitude_state;
    if h<=P.altitude_take_off_zone,     
        altitude_state = 1;
    elseif h<=h_c-P.altitude_hold_zone, 
        altitude_state = 2;
    elseif h>=h_c+P.altitude_hold_zone, 
        altitude_state = 3;
    else
        altitude_state = 4;
    end
    if prev_state ~= altitude_state
        init = true;
        disp(state_names{altitude_state})
    end
    
    % implement state machine
    switch altitude_state,
        case 1,  % in take-off zone
            theta_c = P.take_off_pitch;
            delta_a = 0;
            delta_t = 1;
            
        case 2,  % climb zone
            theta_c = airspeed_with_pitch_hold(Va_c, Va, init, P);
            delta_t = 0.7;
             
        case 3, % descend zone
            theta_c = airspeed_with_pitch_hold(Va_c, Va, init, P);
            delta_t = 0;

        case 4, % altitude hold zone
            theta_c = altitude_hold(h_c, h, init, P);
            delta_t = airspeed_with_throttle_hold(Va_c, Va, init, P);
            
    end
    init = false;
    delta_e = pitch_hold(theta_c, theta, q, init, P);
    delta_r = 0;
    
    % artificially saturation delta_t
    delta_t = sat(delta_t,1,0);
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_TECS
%   - longitudinal autopilot based on total energy control systems
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0,
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        phi_c   = course_hold(chi_c, chi, r, 1, P);

    else
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot based on total energy control
    
    
    delta_e = 0;
    delta_t = 0;
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
end
   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_hold(phi_c, phi, p, init, P)
persistent D error_d1
if init == 1
    D = 0;
    error_d1 = 0;
end
error = phi_c - phi;
D = (2*P.Tau-P.Ts)/(2*P.Tau+P.Ts)*D + 2/(2*P.Tau+P.Ts)*(error-error_d1);
u_unsat = P.kp_phi*error - P.kd_phi*p;
delta_a = sat(u_unsat,P.delta_a_max);
end

function phi_c = course_hold(chi_c, chi, r, init, P)
persistent I error_d1
if init == 1
    I = 0;
    error_d1 = 0;
end
error = chi_c - chi;
I = I + (P.Ts/2)*(error + error_d1);
u_unsat = P.kp_chi*error + P.ki_chi*I;
phi_c = sat(u_unsat,P.e_phi_max);
if P.ki_chi~=0
    I = I + P.Ts/P.ki_chi * (phi_c-u_unsat);
end
end

function delta_e = pitch_hold(theta_c, theta, q, init, P)
persistent D error_d1
if init == 1
    D = 0;
    error_d1 = 0;
end
error = theta_c - theta;
D = (2*P.Tau-P.Ts)/(2*P.Tau+P.Ts)*D + 2/(2*P.Tau+P.Ts)*(error-error_d1);
u_unsat = P.kp_theta*error - P.kd_theta*q;
delta_e = sat(u_unsat,P.delta_e_max);
end

function theta_c = altitude_hold(h_c, h, init, P)
persistent I error_d1
if init == 1
    I = 0;
    error_d1 = 0;
end
error = h_c - h;
I = I + (P.Ts/2)*(error + error_d1);
u_unsat = P.kp_h*error + P.ki_h*I;
theta_c = sat(u_unsat,P.e_theta_max);
if P.ki_h~=0
    I = I + P.Ts/P.ki_h * (theta_c-u_unsat);
end
end

function theta_c = airspeed_with_pitch_hold(Va_c, Va, init, P)
persistent I error_d1
if init == 1
    I = 0;
    error_d1 = 0;
end
error = Va_c - Va;
I = I + (P.Ts/2)*(error + error_d1);
u_unsat = P.kp_v2*error + P.ki_v2*I;
theta_c = sat(u_unsat, 80*pi/180);
if P.ki_v2~=0
    I = I + P.Ts/P.ki_v2 * (theta_c-u_unsat);
end
end

function delta_t = airspeed_with_throttle_hold(Va_c, Va, init, P)
persistent I error_d1
if init == 1
    I = 0;
    error_d1 = 0;
end
throttle_trim = P.u_trim(4);
error = Va_c - Va;
I = I + (P.Ts/2)*(error + error_d1);
u_unsat = throttle_trim + P.kp_v*error + P.ki_v*I;
delta_t = sat(u_unsat, 1, 0);
if P.ki_v~=0
    I = I + P.Ts/P.ki_v * (delta_t-u_unsat);
end
end


  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
if nargin == 2
    low_limit = -up_limit;
end
if in > up_limit,
    out = up_limit;
elseif in < low_limit;
    out = low_limit;
else
    out = in;
end
end
  
 