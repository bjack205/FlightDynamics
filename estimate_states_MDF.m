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
   t             = uu(14);
   
   g = P.gravity;
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %%%%%%%%%%% LPF  Section 8.3 %%%%%%%%%%%%%%%%%
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
   persistent lpf_gyro_x
   persistent lpf_gyro_y
   persistent lpf_gyro_z
   persistent lpf_accel_x
   persistent lpf_accel_y
   persistent lpf_accel_z
   persistent lpf_static_pres
   persistent lpf_diff_pres
   persistent lpf_gps_n
   persistent lpf_gps_e
   persistent lpf_gps_Vg
   persistent lpf_gps_course
   
   if t==0
       lpf_gyro_x = 0;
       lpf_gyro_y = 0;
       lpf_gyro_z = 0;
       lpf_accel_x = 0;
       lpf_accel_y = 0;
       lpf_accel_z = 0;
       lpf_static_pres = P.rho*P.gravity*(-P.pd0);
       lpf_diff_pres = 0.5*P.rho*P.Va0^2;
   end
   
   %%% LPF Angular Rates %%%
   lpf_gyro_x = P.lpf_alpha*lpf_gyro_x + (1 - P.lpf_alpha)*y_gyro_x;
   lpf_gyro_y = P.lpf_alpha*lpf_gyro_y + (1 - P.lpf_alpha)*y_gyro_y;
   lpf_gyro_z = P.lpf_alpha*lpf_gyro_z + (1 - P.lpf_alpha)*y_gyro_z;
   phat = lpf_gyro_x;
   qhat = lpf_gyro_y;
   rhat = lpf_gyro_z;
   
   %%% LPF Altitude from Static Pres %%%
   lpf_static_pres = P.lpf_alpha1*lpf_static_pres + (1 - P.lpf_alpha1)*y_static_pres;
   hhat = lpf_static_pres/P.rho/P.gravity;
   
   %%% LPF Air Speed from diff Pres %%%
   lpf_diff_pres = P.lpf_alpha1*lpf_diff_pres + (1 - P.lpf_alpha1)*y_diff_pres;
   Vahat = sqrt(2*lpf_diff_pres/P.rho);
   
   %%% LPF Roll, Pitch from Accel %%%
   lpf_accel_x = P.lpf_alpha*lpf_accel_x + (1 - P.lpf_alpha)*y_accel_x;
   lpf_accel_y = P.lpf_alpha*lpf_accel_y + (1 - P.lpf_alpha)*y_accel_y;
   lpf_accel_z = P.lpf_alpha*lpf_accel_z + (1 - P.lpf_alpha)*y_accel_z;
   phihat_accel = atan(lpf_accel_y/lpf_accel_z);
   thetahat_accel = asin(lpf_accel_x/P.gravity);
   
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %%%%%% EKF %%%%%%%%%%%%%%%%%%%%%%%%%%%
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
   %%%%%%%%%%%%%%%%%%%%%%%
   % Attitude Estimation %
   %%%%%%%%%%%%%%%%%%%%%%%
   
   % Tested and seems to work great!
   persistent xhat_a 
   persistent y_accel_x_old
   persistent P_a
   
   % Init persistent variables
   if t ==0
       xhat_a = [P.phi0; P.theta0];
       P_a = diag([(15*pi/180)^2, (15*pi/180)^2]);
   end
   
   % Matricies
   R_accel = P.sigma_accel_x^2;
   Q_a = [0.0000001, 0; 0, 0.0000001];
   
   %%%% Prediction %%%%
   N = 10; % Prediction steps
   for i=1:N
       % States
       phi = xhat_a(1);
       theta = xhat_a(2);
   
       % Trigs
       sp = sin(phi);
       cp = cos(phi);
       ct = cos(theta);
       tt = tan(theta);
   
       f_a = [phat + qhat*sp*tt + rhat*cp*tt;...
       qhat*cp - rhat*sp];
   
       df_a = [qhat*cp*tt-rhat*sp*tt, (qhat*sp-rhat*cp)/((ct)^2);...
       -qhat*sp-rhat*cp, 0];
   
       G_a = [ 1, sp*tt, cp*tt; ...
                0, cp, -sp];
   
       xhat_a = xhat_a + (P.Ts/N)*f_a;
       A_a = df_a;
       P_a = P_a + (P.Ts/N)*(A_a*P_a + P_a*A_a' + Q_a);
   end
   
   %%%% Update %%%%
   % Equations
   sp = sin(phi);
   cp = cos(phi);
   ct = cos(theta);
   st = sin(theta);
       
   h_a = [qhat*Vahat*st+g*st;...
       rhat*Vahat*ct-phat*Vahat*st-g*ct*sp;...
       -qhat*Vahat*ct-g*ct*cp];
   
   dh_a = [0, qhat*Vahat*ct+g*ct;...
       -g*cp*ct, -rhat*Vahat*st-phat*Vahat*ct+g*sp*st;...
       g*sp*ct, (qhat*Vahat+g*cp)*st];
   
   if (y_accel_x_old ~= y_accel_x)
       % x-axis accel
       h_i = h_a(1);
       C_i = dh_a(1,:);
       L_a = P_a*C_i'/(R_accel + C_i*P_a*C_i');
       P_a = (eye(2) - L_a*C_i)*P_a;
       xhat_a = xhat_a + L_a*(y_accel_x - h_i);
       % y-axis accel
       h_i = h_a(2);
       C_i = dh_a(2,:);
       L_a = P_a*C_i'/(R_accel + C_i*P_a*C_i');
       P_a = (eye(2) - L_a*C_i)*P_a;
       xhat_a = xhat_a + L_a*(y_accel_y - h_i);
       % z-axis accel
       h_i = h_a(3);
       C_i = dh_a(3,:);
       L_a = P_a*C_i'/(R_accel + C_i*P_a*C_i');
       P_a = (eye(2) - L_a*C_i)*P_a;
       xhat_a = xhat_a + L_a*(y_accel_z - h_i);
   end
   
   % update estimates
   phihat = xhat_a(1);
   thetahat = xhat_a(2);
   
   
   %%%%%%%%%%%%%%%%%%%%%%%
   % GPS Smoother %
   %%%%%%%%%%%%%%%%%%%%%%%
   
   persistent xhat_p 
   persistent P_p
   persistent gps_n_old
   persistent gps_e_old
   persistent gps_Vg_old
   persistent gps_x_old
   
   % Init persistent variables
   if t ==0
       xhat_p = [P.pn0; P.pe0; P.Va0; P.psi0; 0; 0; P.psi0];
%        P_p = diag([10^2, 10^2, 1^2, (10*pi/180)^2, 10^2, 10^2, (5*pi/180)^2]);
       P_p = diag([0.03, 0.03, 0.1^2, (5*pi/180), 0.2^2, 0.22^2, (5*pi/180)]);
       gps_n_old = -9999;
       gps_e_old = -9999;
       gps_Vg_old = -9999;
       gps_x_old = -9999;
   end
   
   % Matricies
   Q_p = diag([0.0001, 0.0001, 0.0001, 0.000001, 0.0001, 0.0001, 0.0001]);
   R_p = diag([P.sigma_gps(1)^2, P.sigma_gps(2)^2, P.gps_sigma_Vg^2, P.gps_sigma_x^2, 0.001, 0.001]);
   
   %%%% Prediction %%%%
   N = 10; % Prediction steps
   for i=1:N
       % States
       pn = xhat_p(1);
       pe = xhat_p(2);
       Vg = xhat_p(3);
       chi = xhat_p(4);
       wn = xhat_p(5);
       we = xhat_p(6);
       psi = xhat_p(7);
   
       % Trigs
       sp = sin(phihat);
       cp = cos(phihat);
       tp = tan(phihat);
       st = sin(thetahat);
       ct = cos(thetahat);
       tt = tan(thetahat);
       cs = cos(psi);
       ss = sin(psi);
       ts = tan(psi);
       cc = cos(chi);
       sc = sin(chi);
   
       psidot = qhat*(sp/ct) + rhat*(cp/ct);
       Vgdot = ((Vahat*cs+wn)*(-Vahat*psidot*ss)+(Vahat*ss+we)*(Vahat*psidot*cs))/(Vg);
       
       f_p = [Vg*cos(chi);...
           Vg*sin(chi);...
           Vgdot;...
           (g/Vg)*tp*cos(chi-psi);...
           0;...
           0;...;
           psidot];
   
       df_p = [...
           0, 0, cc, -Vg*sc, 0, 0, 0;...
           0, 0, sc, Vg*cc, 0, 0, 0;...
           0, 0, -Vgdot/Vg, 0, -psidot*Vahat*ss, psidot*Vahat*cs, (-psidot*Vahat*(wn*cs + we*ss))/Vg;...
           0, 0, (-g*tp*cos(chi-psi))/Vg^2, (-g*tp*sin(chi-psi))/Vg^2, 0, 0, (g*tp*sin(chi-psi))/Vg^2;...
           0, 0, 0, 0, 0, 0, 0;...
           0, 0, 0, 0, 0, 0, 0;...
           0, 0, 0, 0, 0, 0, 0];
   
   
       xhat_p = xhat_p + (P.Ts/N)*f_p;
       A_p = df_p;
       P_p = P_p + (P.Ts/N)*(A_p*P_p + P_p*A_p' + Q_p);
   end
   
   %%%% Update %%%%
   if (y_gps_n ~= gps_n_old) | (y_gps_e ~= gps_e_old) | (y_gps_Vg ~= gps_Vg_old) | (y_gps_course ~= gps_x_old),

       % States
       pn = xhat_p(1);
       pe = xhat_p(2);
       Vg = xhat_p(3);
       chi = xhat_p(4);
       wn = xhat_p(5);
       we = xhat_p(6);
       psi = xhat_p(7);
       
       % gps n
       h_p = xhat_p(1);
       C_p = [1, 0, 0, 0, 0, 0, 0];
       L_p = P_p*C_p'/(R_p(1,1) + C_p*P_p*C_p');
       P_p = (eye(7) - L_p*C_p)*P_p;
       xhat_p = xhat_p + L_p*(y_gps_n - h_p);
       
       % gps e
       h_p = xhat_p(2);
       C_p = [0, 1, 0, 0, 0, 0, 0];
       L_p = P_p*C_p'/(R_p(2,2) + C_p*P_p*C_p');
       P_p = (eye(7) - L_p*C_p)*P_p;
       xhat_p = xhat_p + L_p*(y_gps_e - h_p);
       
       % gps Vg
       h_p = xhat_p(3);
       C_p = [0, 0, 1, 0, 0, 0, 0];
       L_p = P_p*C_p'/(R_p(3,3) + C_p*P_p*C_p');
       P_p = (eye(7) - L_p*C_p)*P_p;
       xhat_p = xhat_p + L_p*(y_gps_Vg - h_p);
       
       % gps X
       % Wrap Gps course
       while (y_gps_course)>pi, y_gps_course = y_gps_course - 2*pi; end
       while (y_gps_course)<-pi, y_gps_course = y_gps_course + 2*pi; end
       h_p = xhat_p(4);
       C_p = [0, 0, 0, 1, 0, 0, 0];
       L_p = P_p*C_p'/(R_p(4,4) + C_p*P_p*C_p');
       P_p = (eye(7) - L_p*C_p)*P_p;
       xhat_p = xhat_p + L_p*(y_gps_course - h_p);
       
       % Pseudo Measurements
       % Pseudo Measurements
       % Wind north
       h_p = Vahat*cos(psi) + wn - Vg*cos(chi);
       C_p = [0, 0, -cos(chi), Vg*sin(chi), 1, 0, -Vahat*sin(psi)];
       L_p = P_p*C_p'/(R_p(5, 5) + C_p*P_p*C_p');
       P_p = (eye(7) - L_p*C_p)*P_p;
       xhat_p = xhat_p + L_p*(0 - h_p); % assume measurement of 0
       
       % Wind east
       h_p = Vahat*sin(psi) + we - Vg*sin(chi);
       C_p = [0, 0, -sin(chi), -Vg*cos(chi), 0, 1, Vahat*cos(psi)];
       L_p = P_p*C_p'/(R_p(6,6) + C_p*P_p*C_p');
       P_p = (eye(7) - L_p*C_p)*P_p;
       xhat_p = xhat_p + L_p*(0 - h_p); % assume measurement of 0
       
   % update persistents
       gps_n_old = y_gps_n;
       gps_e_old = y_gps_e;
       gps_Vg_old = y_gps_Vg;
       gps_x_old = y_gps_course;
   end
   
  % update estimates
   pnhat = xhat_p(1);
   pehat = xhat_p(2);
   Vghat = xhat_p(3);
   chihat = xhat_p(4);
   wnhat = xhat_p(5);
   wehat = xhat_p(6);
   psihat = xhat_p(7);
       
    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
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
end
