function P = GenCoefs(P)
G = P.G;
P.C_p_0 =       G(3)*P.C_ell_0 +       G(4)*P.C_n_0;
P.C_p_beta =    G(3)*P.C_ell_beta +    G(4)*P.C_n_beta;
P.C_p_p =       G(3)*P.C_ell_p +       G(4)*P.C_n_p;
P.C_p_r =       G(3)*P.C_ell_r +       G(4)*P.C_n_r;
P.C_p_delta_a = G(3)*P.C_ell_delta_a + G(4)*P.C_n_delta_a;
P.C_p_delta_r = G(3)*P.C_ell_delta_r + G(4)*P.C_n_delta_r;

P.C_r_0 =       G(4)*P.C_ell_0 +       G(8)*P.C_n_0;
P.C_r_beta =    G(4)*P.C_ell_beta +    G(8)*P.C_n_beta;
P.C_r_p =       G(4)*P.C_ell_p +       G(8)*P.C_n_p;
P.C_r_r =       G(4)*P.C_ell_r +       G(8)*P.C_n_r;
P.C_r_delta_a = G(4)*P.C_ell_delta_a + G(8)*P.C_n_delta_a;
P.C_r_delta_r = G(4)*P.C_ell_delta_r + G(8)*P.C_n_delta_r;


% P.AR = P.b^2/P.S_wing;
% P.sigma = @(alpha) (1+exp(-P.M*(alpha-P.alpha0)) + exp(P.M*(alpha+P.alpha0))) / ...
%     ((1+exp(-P.M*(alpha-P.alpha0))) * (1+exp(P.M*(alpha+P.alpha0))));
% P.C_L = @(alpha) (1-P.sigma(alpha))*(P.C_L_0 + P.C_L_alpha*alpha) + P.sigma(alpha)*(2*sign(alpha)*sin(alpha)^2*cos(alpha));
% P.C_D = @(alpha) P.C_D_p + (P.C_L_0+P.C_L_alpha*alpha)^2/(pi*P.e*P.AR);
% 
% P.C_X = @(alpha)          -P.C_D(alpha)*cos(alpha)  + P.C_L(alpha)*sin(alpha);
% P.C_X_q = @(alpha)        -P.C_D_q*cos(alpha)       + P.C_L_q*sin(alpha);
% P.C_X_delta_e = @(alpha)  -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e*sin(alpha);
% P.C_Z = @(alpha)          -P.C_D(alpha)*sin(alpha)  - P.C_L(alpha)*cos(alpha);
% P.C_Z_q = @(alpha)        -P.C_D_q*sin(alpha)       - P.C_L_q*cos(alpha);
% P.C_Z_delta_e = @(alpha)  -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e*cos(alpha);