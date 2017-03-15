% gps.m
%   Compute the output of gps sensor
%
%  Revised:
%   3/5/2010 - RB 
%   5/14/2010 - RB

function y = gps(uu, P)

    % relabel the inputs
    Va      = uu(1);
%    alpha   = uu(2);
%    beta    = uu(3);
    wn      = uu(4);
    we      = uu(5);
%    wd      = uu(6);
    pn      = uu(7);
    pe      = uu(8);
    pd      = uu(9);
%    u       = uu(10);
%    v       = uu(11);
%    w       = uu(12);
%    phi     = uu(13);
%    theta   = uu(14);
    psi     = uu(15);
%    p       = uu(16);
%    q       = uu(17);
%    r       = uu(18);
    t       = uu(19);
    
 
    % construct North, East, and altitude GPS measurements
    persistent vn ve vd
    if t == 0
        vn = zeros(2,1);
        ve = zeros(2,1);
        vd = zeros(2,1);
    end
    k_gps = 1/1100;
    
    Ts = 1;
    vn(2) = exp(-k_gps*Ts)*vn(1) + P.sigma_gps(1)*randn;
    ve(2) = exp(-k_gps*Ts)*ve(1) + P.sigma_gps(2)*randn;
    vd(2) = exp(-k_gps*Ts)*vd(1) + P.sigma_gps(3)*randn;
    vn(1) = vn(2);
    ve(1) = ve(2);
    vd(1) = vd(2);
    
    y_gps_n = pn + vn(2);
    y_gps_e = pe + ve(2); 
    y_gps_h = -pd + vd(2); 
    
    % construct groundspeed and course measurements
    sigma_v = 0.05;
    sigma_Vg = sigma_v;
    
    Vn = Va*cos(psi)+wn;
    Ve = Va*sin(psi)+we;
    Vg = sqrt(Vn^2+Ve^2);
    sigma_chi = sigma_v / Vg;
    y_gps_Vg     = sqrt((Va*cos(psi)+wn^2)^2 + (Va*sin(psi)+we)^2) + randn*sigma_Vg;
    y_gps_course = atan2(Va*sin(psi)+we, Va*cos(psi)+wn) + randn*sigma_chi;

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
    
end



