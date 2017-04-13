function [T,P] = compute_tf_model(x_trim,u_trim,P)
% x_trim is the trimmed state,
% u_trim is the trimmed input

% add stuff here
pn    = x_trim(1);
pe    = x_trim(2);
pd    = x_trim(3);
u     = x_trim(4);
v     = x_trim(5);
w     = x_trim(6);
phi   = x_trim(7);
theta = x_trim(8);
psi   = x_trim(9);
p     = x_trim(10);
q     = x_trim(11);
r     = x_trim(12);
delta_e = u_trim(1);
delta_a = u_trim(2);
delta_r = u_trim(3);
delta_t = u_trim(4);

Va_trim = sqrt(u^2+v^2+w^2);
theta_trim = theta;
gamma = theta;

thetaddot = 0;
qdot = 0;

a_phi1 = -1/2*P.rho*Va_trim^2*P.S_wing*P.b*P.C_p_p*P.b/(2*Va_trim);
a_phi2 =  1/2*P.rho*Va_trim^2*P.S_wing*P.b*P.C_p_delta_a;
d_phi2 = 1/Va_trim*(p*w-r*u+P.g*cos(theta)*sin(phi)) + (P.rho*Va_trim*P.S_wing/(2*P.mass))*(P.C_Y_0 + P.C_Y_p*P.b*p/(2*Va_trim) + P.C_Y_r*P.b*r/(2*Va_trim) + P.C_Y_delta_a*delta_a);

pVaS2m = P.rho*Va_trim*P.S_wing/(2*P.mass);
a_beta1 = pVaS2m*P.C_Y_beta;
a_beta2 = pVaS2m*P.C_Y_delta_r;
d_beta = 1/Va_trim*(p*w-r*u + P.g*cos(theta)*sin(phi)) + pVaS2m*(P.C_Y_0 + P.C_Y_p*P.b*p/(2*Va_trim) + P.C_Y_r*P.b*r/(2*Va_trim) + P.C_Y_delta_a*delta_a);

pVacS = P.rho*Va_trim^2*P.c*P.S_wing/(2*P.Jy);
a_theta1 = -pVacS*P.C_m_q*P.c/(2*Va_trim);
a_theta2 = -pVacS*P.C_m_alpha;
a_theta3 =  pVacS*P.C_m_delta_e;
d_theta1 = q*cos(phi-1)-r*sin(phi);
d_theta_dot = thetaddot - qdot;
d_theta2 =  P.G(6)*(r^2-p^2) + P.G(5)*p*r + pVacS*(P.C_m_0 - P.C_m_alpha*gamma - P.C_m_q*P.c/(2*Va_trim)*d_theta1)+d_theta_dot;

a_V1 = P.rho*Va_trim*P.S_wing/P.mass*(P.C_D_0 + P.C_D_alpha + P.C_D_delta_e*delta_e) + P.rho*P.S_prop/P.mass*P.C_prop*Va_trim;
a_V2 = P.rho*P.S_prop/P.mass*P.C_prop*P.k_motor^2*delta_t;
a_V3 = P.g*cos(theta_trim-psi);

    
% define transfer functions
T.phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T.chi_phi       = tf([P.g/Va_trim],[1,0]);
T.theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T.h_theta       = tf([Va_trim],[1,0]);
T.h_Va          = tf([theta_trim],[1,0]);
T.Va_delta_t    = tf([a_V2],[1,a_V1]);
T.Va_theta      = tf([-a_V3],[1,a_V1]);
T.v_delta_r     = tf([Va_trim*a_beta2],[1,a_beta1]);

%% Compute gains
% Roll Attitude
P.kp_phi = P.delta_a_max/P.e_phi_max*sign(a_phi2);
P.wn_phi = sqrt(abs(a_phi2)*P.delta_a_max/P.e_phi_max);
P.kd_phi = (2*P.zeta_phi*P.wn_phi-a_phi1)/a_phi2;

% Course Hold
Vg = Va_trim;
P.wn_chi = P.wn_phi/P.W_chi;
P.kp_chi = 2*P.zeta_chi*P.wn_chi*Vg/P.g;
P.ki_chi = P.wn_chi^2*Vg/P.g;

% Sideslip Hold
P.kp_beta = P.delta_r_max/P.e_beta_max*sign(a_beta2);
P.ki_beta = ((a_beta1+a_beta2*P.kp_beta)/(2*P.zeta_beta))^2/a_beta2;

% Pitch Attitude
P.kp_theta = P.delta_e_max/P.e_theta_max*sign(a_theta3);
P.wn_theta = sqrt(a_theta2 + P.delta_e_max/P.e_theta_max*abs(a_theta3));
P.kd_theta = -(2*P.zeta_theta*P.wn_theta - a_theta1)/a_theta3;
P.Kdc_theta = P.kp_theta*a_theta3/(a_theta2 + P.kp_theta*a_theta3);

% Altitude Hold
P.wn_h = P.wn_theta/P.W_h;
P.ki_h = P.wn_h^2/(P.Kdc_theta*Va_trim);
P.kp_h = 2*P.zeta_h*P.wn_h/(P.Kdc_theta*Va_trim);

% Airspeed Hold Pitch
P.wn_v2 = P.wn_theta/P.W_v2;
P.ki_v2 = -P.wn_v2^2/(P.Kdc_theta*P.g);
P.kp_v2 = (a_V1 - 2*P.zeta_v2*P.wn_v2)/(P.Kdc_theta*P.g);

% Airspeed Hold Throttle
P.ki_v = P.wn_v^2^2/a_V2;
P.kp_v = (2*P.zeta_v*P.wn_v - a_V1)/a_V2;

%% Gains for Follower
% Roll Attitude
P.F.kp_phi = P.F.delta_a_max/P.F.e_phi_max*sign(a_phi2);
P.F.wn_phi = sqrt(abs(a_phi2)*P.F.delta_a_max/P.F.e_phi_max);
P.F.kd_phi = (2*P.F.zeta_phi*P.F.wn_phi-a_phi1)/a_phi2;

% Course Hold
Vg = Va_trim;
P.F.wn_chi = P.F.wn_phi/P.F.W_chi;
P.F.kp_chi = 2*P.F.zeta_chi*P.F.wn_chi*Vg/P.g;
P.F.ki_chi = P.F.wn_chi^2*Vg/P.g;

% Pitch Attitude
P.F.kp_theta = P.F.delta_e_max/P.F.e_theta_max*sign(a_theta3);
P.F.wn_theta = sqrt(a_theta2 + P.F.delta_e_max/P.F.e_theta_max*abs(a_theta3));
P.F.kd_theta = -(2*P.F.zeta_theta*P.F.wn_theta - a_theta1)/a_theta3;
P.F.Kdc_theta = P.F.kp_theta*a_theta3/(a_theta2 + P.F.kp_theta*a_theta3);

