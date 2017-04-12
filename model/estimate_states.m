% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position,
%   pehat    - estimated East position,
%   hhat     - estimated altitude,
%   Vahat    - estimated airspeed,
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle,
%   thetahat - estimated pitch angel,
%   chihat   - estimated course,
%   phat     - estimated roll rate,
%   qhat     - estimated pitch rate,
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed,
%   wnhat    - estimate of North wind,
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
%
%
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%

function xhat = estimate_states(uu, P)

% rename inputs
y_gyro_x      = uu(1);
y_gyro_y      = uu(2);
y_gyro_z      = uu(3);
y_accel_x     = uu(4);
y_accel_y     = uu(5);
y_accel_z     = uu(6);
y_static_pres = uu(7);
y_diff_pres   = uu(8);
y_gps_n       = uu(9);
y_gps_e       = uu(10);
y_gps_h       = uu(11);
y_gps_Vg      = uu(12);
y_gps_course  = uu(13);
plane         = uu(14);

if length(uu) == 14
    plane = 1;
    t = uu(14);
else
    t = uu(15);
end

% Break into sections by rate
uu_a = uu(1:8);
uu_gps = uu(9:13);

% Pseudo measurements
y_wind_n = 0;
y_wind_e = 0;

% not estimating these states
alphahat = 0;
betahat  = 0;
bxhat    = 0;
byhat    = 0;
bzhat    = 0;

persistent xhat_d1 uu_a_d1 uu_gps_d1 P_a P_gps
persistent lpf_static_pres lpf_diff_press

if t == 0
    
    pnhat = P.pn0; 
    pehat = P.pe0; 
    hhat = -P.pd0;
    Vahat = P.Va0;
    phihat = P.phi0; 
    thetahat = P.theta0; 
    psihat = P.psi0;
    phat = P.p0; 
    qhat = P.q0; 
    rhat =P.r0;
    Vghat = P.Va0;
    wnhat = P.wind_n;
    wehat = P.wind_e;
    psihat = P.psi0;
    chihat = P.psi0;
    
    P_a(:,:,plane) = diag([5*pi/180; 5*pi/180].^2);
    %P_gps = diag([5 5 10 0.01 10 10 0.01].^2);
    P_gps(:,:,plane) = diag([0.03, 0.03, 0.1^2, (5*pi/180), 0.2^2, 0.22^2, (5*pi/180)]);
    
    uu_a_d1(:,plane) = ones(8,1)*-100;
    uu_gps_d1(:,plane) = ones(5,1)*-100;
    
    xhat_d1(:,plane) = [...
        pnhat;...1
        pehat;...2
        hhat;...3
        Vahat;...4
        alphahat;...5
        betahat;...6
        phihat;...7
        thetahat;...8
        chihat;...9
        phat;...10
        qhat;...11
        rhat;...12
        Vghat;...13
        wnhat;...14
        wehat;...15
        psihat;...16
        bxhat;...17
        byhat;...18
        bzhat;...19
        ];
    
    lpf_static_pres(:,plane) = P.rho*P.g*(-P.pd0);
    lpf_diff_press(:,plane) = 0.5*P.rho*P.Va0^2;
else
    
    pnhat    = xhat_d1(1,plane);
    pehat    = xhat_d1(2,plane);
    hhat     = xhat_d1(3,plane);
    Vahat    = xhat_d1(4,plane);
    phihat   = xhat_d1(7,plane);
    thetahat = xhat_d1(8,plane);
    chihat   = xhat_d1(9,plane);
    phat     = xhat_d1(10,plane);
    qhat     = xhat_d1(11,plane);
    rhat     = xhat_d1(12,plane);
    Vghat    = xhat_d1(13,plane);
    wnhat    = xhat_d1(14,plane);
    wehat    = xhat_d1(15,plane);
    psihat   = xhat_d1(16,plane);
    
end

% Filter Sensors Data
alpha_pres = 0;
lpf_static_pres(:,plane) = alpha_pres*lpf_static_pres(:,plane) + (1-alpha_pres)*y_static_pres;
lpf_diff_press(:,plane) = alpha_pres*lpf_diff_press(:,plane) + (1-alpha_pres)*y_diff_pres;

% Get values from sensors
p = y_gyro_x;
q = y_gyro_y;
r = y_gyro_z;
h = lpf_static_pres(:,plane)/(P.rho*P.g);
Va = sqrt(2/P.rho*lpf_diff_press(:,plane));
pn = y_gps_n;
pe = y_gps_e;
chi = y_gps_course;
Vg = y_gps_Vg;

% Gains
Q_a = diag([1e-8 1e-8]);
a = 1e1;
b = 1e-2;
Q_gps = diag([a a a b a a b]);
%Q_gps = diag([0.0001, 0.0001, 0.0001, 0.000001, 0.0001, 0.0001, 0.0001]);

% Noise Matrices
P.sigma_v = P.sigma_gps_v;
sigma_Vg = P.sigma_v;
Vn = Va*cos(psihat)+wnhat;
Ve = Va*sin(psihat)+wehat;
Vg = sqrt(Vn^2+Ve^2);
sigma_chi = P.sigma_v / Vg;

Ri_a = diag([P.sigma_accel_x P.sigma_accel_y P.sigma_accel_z].^2);
Ri_gps = diag([P.sigma_gps(1) P.sigma_gps(2) sigma_Vg sigma_chi 1e-6 1e-6].^2);
%Ri_gps = diag([P.sigma_gps(1)^2, P.sigma_gps(2)^2, sigma_Vg^2, sigma_chi^2, 0.001, 0.001]);


N = P.Ts/P.Ts_estimator;

%% Attitude Estimation
xhat_a = [phihat; thetahat];
for i = 1:N
    f_a = [phat+qhat*sin(phihat)*tan(thetahat) + rhat*cos(phihat)*tan(thetahat);...
           qhat*cos(phihat)-rhat*sin(phihat)];
    
    xhat_a = xhat_a + P.Ts_estimator*f_a;
    phihat = xhat_a(1);
    thetahat = xhat_a(2);
    
    df_a = [qhat*cos(phihat)*tan(thetahat)-rhat*sin(phihat)*tan(thetahat)      (qhat*sin(phihat)-rhat*cos(phihat))/(cos(thetahat)^2);...
            -qhat*sin(phihat)-rhat*cos(phihat)                                                      0                        ];
    A = df_a;
    P_a(:,:,plane) = P_a(:,:,plane) + P.Ts_estimator*(A*P_a(:,:,plane) + P_a(:,:,plane)*A' + Q_a);
end

% Measurement update
if any(abs(uu_a - uu_a_d1(:,plane)))
    h_a = [q*Va*sin(thetahat) + P.g*sin(thetahat);...
           r*Va*cos(thetahat) - p*Va*sin(thetahat)-P.g*cos(thetahat)*sin(phihat);...
           -q*Va*cos(thetahat) - P.g*cos(thetahat)*cos(phihat)];
    dh_a = [0                               q*Va*cos(thetahat)+P.g*Va*cos(thetahat)+P.g*cos(thetahat);...
            -P.g*cos(phihat)*cos(thetahat) -r*Va*sin(thetahat)-p*Va*cos(thetahat)+P.g*sin(phihat)*sin(thetahat);...
             P.g*sin(phihat)*cos(thetahat) (q*Va+P.g*cos(phihat))*sin(thetahat)];
    y = [y_accel_x; y_accel_y; y_accel_z];
    
    Ci_a = dh_a;
    Li_a = P_a(:,:,plane)*Ci_a'*inv(Ri_a+Ci_a*P_a(:,:,plane)*Ci_a');
    P_a(:,:,plane) = (eye(2)-Li_a*Ci_a)*P_a(:,:,plane);
    xhat_a = xhat_a + Li_a*(y - h_a);
end

%% GPS Smoothing
xhat_gps = [pnhat; pehat; Vghat; chihat; wnhat; wehat; psihat];
gate_gps = [10; 10; 20; 20*pi/180; 10; 10; 20*pi/180];

for i = 1:N
    psidot = qhat*sin(phihat)/cos(thetahat) + rhat*cos(phihat)/cos(thetahat);
    Vgdot = ((Vahat*cos(psihat)+wnhat)*(-Vahat*psidot*sin(psihat)) + (Vahat*sin(psihat)+wnhat)*(Vahat*psidot*cos(psihat)))/Vghat;
    
    f_gps = [Vghat*cos(chihat);...
             Vghat*sin(chihat);...
             ((Vahat*cos(psihat)+wnhat)*(-Vahat*psidot*sin(psihat)) + (Vahat*sin(psihat)+wehat)*(Vahat*psihat*cos(psihat)))/Vghat;...
             P.g/Vghat*tan(phihat)*cos(chihat-psihat);...
             0;...
             0;...
             qhat*sin(phihat)/cos(thetahat) + rhat*cos(phihat)/cos(thetahat)];

         
    xhat_gps = xhat_gps + P.Ts_estimator*f_gps;
    pnhat = xhat_gps(1);
    pehat = xhat_gps(2);
    Vghat = xhat_gps(3);
    chihat = xhat_gps(4);
    wnhat = xhat_gps(5);
    wehat = xhat_gps(6);
    psihat = xhat_gps(7);
    
    dVgdot_dpsi = -psidot*Vahat*(wnhat*cos(psihat)+wehat*sin(psihat))/Vghat;
    dchidot_dVg = -P.g/(Vghat^2)*tan(phihat)*cos(chihat-psihat);
    dchidot_dchi = -P.g/Vghat*tan(phihat)*sin(chihat-psihat);
    dchidot_dpsi =  P.g/Vghat*tan(phihat)*sin(chihat-psihat);
    df_gps = [0 0 cos(chihat)  -Vghat*sin(chihat) 0 0 0;...
              0 0 sin(chihat)   Vghat*cos(chihat) 0 0 0;...
              0 0 -Vgdot/Vghat  0                 -psidot*Vahat*sin(psihat) psidot*Vahat*cos(psihat) dVgdot_dpsi;
              0 0 dchidot_dVg       dchidot_dchi       0 0 dchidot_dpsi;...
              0 0 0 0 0 0 0;...
              0 0 0 0 0 0 0;...
              0 0 0 0 0 0 0];
    A = df_gps;
    P_gps(:,:,plane) = P_gps(:,:,plane) + P.Ts_estimator*(A*P_gps(:,:,plane) + P_gps(:,:,plane)*A' + Q_gps);
end

% Measurement update
if any(abs(uu_gps - uu_gps_d1(:,plane)))
    h_gps = [pnhat; pehat; Vghat; chihat;...
             Va*cos(psihat) + wnhat - Vghat*cos(chihat);...
             Va*sin(psihat) + wehat - Vghat*sin(chihat)];
    dh_gps = [1 0 0 0 0 0 0;...
              0 1 0 0 0 0 0;...
              0 0 1 0 0 0 0;...
              0 0 0 1 0 0 0;...
              0 0 -cos(chihat) Vg*sin(chi) 1 0 -Va*sin(psihat);...
              0 0 -sin(chihat) Vg*cos(chi) 0 1  Va*cos(psihat)];
     
    Ci_gps = dh_gps;
    Li_gps = P_gps(:,:,plane)*Ci_gps'*inv(Ri_gps + Ci_gps*P_gps(:,:,plane)*Ci_gps');
    P_gps(:,:,plane) = (eye(7)-Li_gps*Ci_gps)*P_gps(:,:,plane);
    
    y_gps = [y_gps_n; y_gps_e; y_gps_Vg; y_gps_course; y_wind_n; y_wind_e];
    err = y_gps - h_gps;
    if abs(y_gps(4) - h_gps(4)) > pi
        err(4) = atan2(sin(err(4)),cos(err(4)));
    end
    
    update = Li_gps*(err);
    xhat_gps = xhat_gps + update;
    
    pnhat  = xhat_gps(1);
    pehat  = xhat_gps(2);
    Vghat  = xhat_gps(3);
    chihat = atan2(sin(xhat_gps(4)),cos(xhat_gps(4)));
    wnhat  = xhat_gps(5);
    wehat  = xhat_gps(6);
    psihat = xhat_gps(7);
    
end

% Low pass filter these?
alpha = 0.8;
alpha_Va = 0.8;
alpha_h = 0.8;
hhat = xhat_d1(3,plane)*alpha_h + (1-alpha_h)*h;
phat = xhat_d1(10,plane)*alpha + (1-alpha)*p;
qhat = xhat_d1(11,plane)*alpha + (1-alpha)*q;
rhat = xhat_d1(12,plane)*alpha + (1-alpha)*r;
Vahat = xhat_d1(4,plane)*alpha_Va + (1-alpha_Va)*Va;

uu_a_d1(:,plane) = uu_a;
uu_gps_d1(:,plane) = uu_gps;
xhat = [...
    pnhat;...1
    pehat;...2
    hhat;...3
    Vahat;...4
    alphahat;...5
    betahat;...6
    phihat;...7
    thetahat;...8
    chihat;...9
    phat;...10
    qhat;...11
    rhat;...12
    Vghat;...13
    wnhat;...14
    wehat;...15
    psihat;...16
    bxhat;...17
    byhat;...18
    bzhat;...19
    ];
xhat_d1(:,plane) = xhat;
end
