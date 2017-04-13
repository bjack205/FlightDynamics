function out = plane_follow(in, P)
% Given the state of the lead aircraft and the current aircraft, return the required commands to
% track a given offset

NN = 19;
x_lead = in(1:NN);

x = in((1:NN) + NN);
NN = NN*2;
offset = in((1:3) + NN);


Va_lead = x_lead(4);
psi_lead = x_lead(9);
chi_lead = atan2(Va_lead*sin(psi_lead)+P.wind_e, Va_lead*cos(psi_lead)+P.wind_n);

switch P.follow_frame
    case 'body'
        % In leader body frame
        
        R = quat2rmat(euler2quat(x_lead(7:9)'));
        x_d(1:3) = R*offset.*[1;1;-1] + x_lead(1:3);
    case 'vehicle-1'
        % In leader vehicle-1 frame
        x_d(1:3) = x_lead(1:3) + offset.*[1;1;-1];
end

delta_n = x_d(1) - x(1);
delta_e = x_d(2) - x(2);
L = sqrt(delta_n^2 + delta_e^2);
chi_f = atan2(delta_e,delta_n);
xi = chi_f - chi_lead;

delta_x = cos(xi)*L; 
delta_y = sin(xi)*L;

if delta_x > P.delta_x_max
    Va_c = P.Va_max;
elseif delta_x < - P.delta_x_max
    Va_c = P.Va_min;
else
    delta_Va = min(P.Va_max - Va_lead, Va_lead - P.Va_min);
    a = delta_Va/((P.delta_x_max)^P.n_Va);
    Va_c = a*(delta_x)^P.n_Va + Va_lead;
    k = delta_Va/P.delta_x_max;
    Va_c = k*delta_x + Va_lead;
end

chi_lead = chi_lead;
chi_c = chi_lead + P.chi_inf_follow*2/pi*atan(P.k_follow*delta_y);
if chi_c > 90*pi/180
    a = 1;
end

h_c = x_d(3);
phi_ff = 0;

% Va_c = P.Va0;
% h_c = 100;
% chi_c = 0;

out = [Va_c; h_c; chi_c; phi_ff];