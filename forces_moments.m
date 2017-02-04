% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    e0      = x(7);
    e1      = x(8);
    e2      = x(9);
    e3      = x(10);
    p       = x(11);
    q       = x(12);
    r       = x(13);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % Get the rotation from the body frame to the vehicle frame
    e = x(7:10);
    R = quat2rmat(e');
    
    %% Compute wind data in NED
    % Convert wind to body frame
    Vgust = R'*[u_wg; v_wg; w_wg]*0;
    Vws_b = R'*[w_ns;w_es;w_ds];
    Vw_b = Vws_b + Vgust;
    
    % compute air data
    Vg_b = [u;v;w];
    Va_b = Vg_b - Vw_b;
    Va = norm(Va_b);
    alpha = atan2(Va_b(3),Va_b(1));
    if Va ~= 0
        beta = asin(Va_b(2)/Va);
    else
        beta = 0;
    end
    
    % compute external forces and torques on aircraft
    
    % Gravity force
    mg = P.mass*P.g;
    F_g = mg*[2*(e1*e3);...
            2*(e2*e3+e1*e0);...
            e3^2+e0^2-e1^2-e2^2];

    % Aerodynamic Forces
    AR = P.b^2/P.S_wing;
    sigma = @(alpha) (1+exp(-P.M*(alpha-P.alpha0)) + exp(P.M*(alpha+P.alpha0))) / ...
        ((1+exp(-P.M*(alpha-P.alpha0))) * (1+exp(P.M*(alpha+P.alpha0))));
    C_L = @(alpha) (1-sigma(alpha))*(P.C_L_0 + P.C_L_alpha*alpha) + sigma(alpha)*(2*sign(alpha)*sin(alpha)^2*cos(alpha));
    C_D = @(alpha) P.C_D_p + (P.C_L_0+P.C_L_alpha*alpha)^2/(pi*P.e*AR);

    C_X =           -C_D(alpha)*cos(alpha)    + C_L(alpha)*sin(alpha);
    C_X_q =         -P.C_D_q*cos(alpha)       + P.C_L_q*sin(alpha);
    C_X_delta_e =   -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e*sin(alpha);
    C_Z =           -C_D(alpha)*sin(alpha)  - C_L(alpha)*cos(alpha);
    C_Z_q =         -P.C_D_q*sin(alpha)       - P.C_L_q*cos(alpha);
    C_Z_delta_e =   -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e*cos(alpha);

    if Va ~= 0
        F_aero = 1/2*P.rho*Va^2*P.S_wing*[...
            C_X + C_X_q*P.c/(2*Va)*q + C_X_delta_e*delta_e;...
            P.C_Y_0 + P.C_Y_beta*beta * P.C_Y_p*P.b/(2*Va)*r + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r;...
            C_Z + C_Z_q*P.c/(2*Va)*q + C_Z_delta_e*delta_e];
    else
        F_aero = zeros(3,1);
    end

    F_prop = 1/2*P.rho*P.S_prop*P.C_prop*[(P.k_motor*delta_t)^2 - Va^2; 0; 0];

    Force = F_g + F_aero + F_prop;

    if Va ~= 0
        Torque = 1/2*P.rho*Va^2*P.S_wing*[...
            P.b*(P.C_ell_0 + P.C_ell_beta*beta + P.C_ell_p*P.b/(2*Va)*r + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r);...
            P.c*(P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*P.c/(2*Va)*q + P.C_m_delta_e*delta_e);...
            P.b*(P.C_n_0 + P.C_n_beta*beta + P.C_n_p*P.b/(2*Va)*p + P.C_n_r*P.b/(2*Va)*r + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r)]...
            +...
            [-P.k_T_P*(P.k_Omega*delta_t)^2; 0; 0];
    else
        Torque = zeros(3,1);
    end
    

    out = [Force; Torque; Va; alpha; beta; R*Vw_b];
end



