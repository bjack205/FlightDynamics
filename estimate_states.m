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

function xhat_out = estimate_states(uu, P)

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
t             = uu(14);

% not estimating these states
alphahat = 0;
betahat  = 0;
bxhat    = 0;
byhat    = 0;
bzhat    = 0;

persistent xhat_d1 uu_d1 P_a P_gps

if t == 0
    pnhat = P.pn0; 
    pehat = P.pe0; 
    hhat = -P.pd0;
    Vahat = P.Vahat;
    phihat = P.phi0; 
    thetahat = P.theta0; 
    psihat = P.psi0;
    phat = P.p0; 
    qhat = P.q0; 
    rhat =P.r0;
    Vahat = P.Va0;
    psihat = P.psi0;
else
    phat = y_gyro_x;
    qhat = y_gyro_y;
    rhat = y_gryo_z;
    hhat = y_static_pres/(P.rho*P.g);
    Vahat = sqrt(2/P.rho*y_diff_pres);
    pnhat = y_gps_n;
    pehat = y_gps_e;
    chihat = y_gps_course;
    Vghat = y_gps_Vg;
    
    pn = xhat_d1(1);
    pe = xhat_d1(2);
    h  = xhat_d1(3);
    Va = xhat_d1(4);
    phi = xhat_d1(7);
    theta = xhat_d1(8);
    chi = xhat_d1(9);
    p = xhat_d1(10);
    q = xhat_d1(11);
    r = xhat_d1(12);
    Vg = xhat_d1(13);
    wn = xhat_d1(14);
    we = xhat_d1(15);
    psi = xhat_d1(16);
    
end
N = P.Ts/P.Ts_estimator;

%% Attitude Estimation
xhat_a = [phihat; thetahat];
for i = 1:N
    f_a = [phat+qhat*sin(phihat)*tan(thetahat) + rhat*cos(phihat)*tan(thetahat);...
           qhat*cos(phihat)-rhat*sin(phihat)];
    
    
    xhat_a = xhat_a + P.Ts_estimator*f_a;
    phihat = xhat_a(1);
    thetahat = xhat_a(2);
    
    df_a = [qhat*cos(phihat)*tan(thetahat)-rhat*sin(phihat)*tan(thetahat)      (qhat*sin(phihat)-r*cos(phi))/cos(thetahat)^2;...
            -qhat*sin(phihat)-rhat*cos(phihat)                                                      0                        ];
    A = df_a;
    P_a = P_a + P.Ts_estimator*(A*P_a + P_a*A' + P.Q_a);
end

% Measurement update
if any(abs(uu_d1-uu))
    h_a = [qhat*Vahat*sin(thetahat) + P.g*sin(thetahat);...
           rhat*Vahat*cos(thetahat) - phat*Vahat*sin(thetahat)-P.g*cos(thetahat)*sin(phihat);...
           -qhat*Vahat*cos(thetahat) - P.g*cos(thetahat)*cos(phihat)];
    dh_a = [0                               qhat*Vahat*cos(thetahat)+P.g*Vahat*cos(thetahat)+P.g*cos(thetahat);...
            -P.g*cos(phihat)*cos(thetahat) -rhat*Vahat*sin(thetahat)-phat*Vahat*cos(thetahat)+P.g*sin(phihat)*sin(thetahat);...
             P.g*sin(phihat)*cos(thetahat) (qhat*Vahat+P.g*cos(phihat))*sin(thetahat)];
    y = [y_accel_x; y_accel_y; y_accel_z];
    
    Ci = dh_a;
    Li = P*Ci'*inv(Ri+Ci*P*Ci');
    P = (I-Li*Ci)*P;
    xhat_a = xhat_a + Li*(y - h_a);
    phihat = xhat_a(1);
    thetahat = xhat_a(2);
end

%% GPS Smoothing
xhat_gps = [pnhat; pehat; Vghat; chihat; wnhat; wehat; psihat];

for i = 1:N
    psidot = qhat*sin(phihat)/cos(thetahat) + rhat*cos(phihat)/cos(thetahat);
    Vgdot = ((Vahat*cos(psihat)+wnhat)*(-Vahat*psidot*sin(psihat)) + (Vahat*sin(psihat)+wnhat)*(Vahat*psidot*cos(psihat)))/Vghat;
    
    f_gps = [Vghat*cos(chihat);...
             Vghat*sin(chihat);...
             ((Vahat*cos(psihat)+wnhat)*(-Vahat*psidot*sin(psihat)) + (Vahat*sin(psihat)+wehat)*(Vahat*psihat*cos(psihat)))/Vghat;...
             P.g/Vghat*tan(psi)*cos(chihat-psihat);...
             0;...
             0;...
             qhat*sin(psihat)/cos(thetahat) + rhat*cos(psihat)/cos(thetahat)];

         
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
              0 0 0 0 0 0 0];
    A = df_gps;
    P_gps = P_gps + P.Ts_estimator*(A_*P_gps + P_gps*A' + P.Q_gps);
end
    

uu_d1 = uu;
xhat = [...
    pnhat;...
    pehat;...
    hhat;...
    Vahat;...
    alphahat;...
    betahat;...
    phihat;...
    thetahat;...
    chihat;...
    phat;...
    qhat;...
    rhat;...
    Vghat;...
    wnhat;...
    wehat;...
    psihat;...
    bxhat;...
    byhat;...
    bzhat;...
    ];
xhat_d1 = xhat;
end
