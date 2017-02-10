function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P)
% x_trim is the trimmed state,
% u_trim is the trimmed input

% add stuff here
pn    = x_trim(1);
pe    = x_trim(2);
pd    = x_trim(3);
u     = x_trim(4);
v     = x_trim(5);
w     = x_trim(6);
e     = x_trim(7:10);
p     = x_trim(11);
q     = x_trim(12);
r     = x_trim(13);
delta_e = u_trim(1);
delta_a = u_trim(2);
delta_r = u_trim(3);
delta_t = u_trim(4);

[phi,theta,psi] = quat2euler(e');
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

pVacS = -P.rho*Va_trim^2*P.c*P.S_wing/(2*P.Jy);
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
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([P.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([Va_trim*a_beta2],[1,a_beta1]);

