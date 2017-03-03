addpath('utils')
addpath('fncs')
addpath('media')
P.g = 9.8;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%% Physical parameters of airframe
P.mass = 25;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;

% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

P.G = GenGammas(P.Jx,P.Jy,P.Jz,P.Jxz);
P = GenCoefs(P);

% wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;

% Actuator Limits
P.delta_a_max = 45*pi/180;
P.delta_r_max = 45*pi/180;
P.delta_e_max = 45*pi/180;

% Sensor data
P.sigma_gyro_x = 0;
P.sigma_gyro_y = 0;
P.sigma_gyro_z = 0;
P.sigma_accel_x = 0;
P.sigma_accel_y = 0;
P.sigma_accel_z = 0;
P.sigma_p_abs = 0;
P.sigma_p_diff = 0;
P.beta_p_abs = 0;
P.beta_p_diff = 0;
P.sigma_Vg = 0;
P.sigma_chi = 0;
P.Ts_gps = 1;

%% Compute trim conditions using 'mavsim_trim.mdl'
% initial airspeed
P.Va0 = 35;
gamma = 00*pi/180;  % desired flight path angle (radians)
R     = Inf ;         % desired radius (m) - use (+) for right handed orbit, 

% autopilot sample rate
P.Ts = 0.01;
P.lambda = 100;
P.Tau = 0.05;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

                    %                          (-) for left handed orbit

%% Run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate



% linearize the equations of motion around trim conditions
%[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

%% Compute Gains

%%%%%% Successive Loop Closure %%%%%%
% Longitudinal Control
P.altitude_take_off_zone = 0;    
P.altitude_hold_zone = 30;       
P.take_off_pitch = 45*pi/180;    
                    

% Roll attitude
P.e_phi_max = 60*pi/180;
P.zeta_phi = 1.2;

% Course Hold
P.W_chi = 20; % >5
P.zeta_chi = 1;

% Sideslip Hold
P.e_beta_max = 15*pi/180;
P.zeta_beta = 0.707;

% Pitch attitude
P.e_theta_max = 45*pi/180;
P.zeta_theta = 0.9;

% Altitude hold
P.W_h = 30; % 15>x>5
P.zeta_h = 1;

% Airspeed hold pitch
P.W_v2 = 6;
P.zeta_v2 = 1;

% Airspeed hold throttle
P.wn_v = 1;
P.zeta_v = 0.9;

% compute different transfer functions
[T,P]= compute_tf_model(x_trim,u_trim,P);

%%%%%% Total Energy Control %%%%%%
P.h_e = 20;  

P.kp_E = 1;
P.kd_E = 2;
P.ki_E = 0.5;

P.kp_B = 2.5;
P.kd_B = 1.5;
P.ki_B = 0.5;


