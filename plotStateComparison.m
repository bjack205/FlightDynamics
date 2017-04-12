function plotStateComparison(uu)
%
% modified 12/11/2009 - RB
isquat = 0;
NN = 53;
N = (length(uu)-1)/NN;
% process inputs to function
for i = 1:N
    % process inputs to function
    pn(i)          = uu(1+(i-1)*NN);             % North position (meters)
    pe(i)          = uu(2+(i-1)*NN);             % East position (meters)
    h(i)           = -uu(3+(i-1)*NN);            % altitude (meters)
    u(i)           = uu(4+(i-1)*NN);             % body velocity along x-axis (meters/s)
    v(i)           = uu(5+(i-1)*NN);             % body velocity along y-axis (meters/s)
    w(i)           = uu(6+(i-1)*NN);             % body velocity along z-axis (meters/s)
    phi(i)         = 180/pi*uu(7+(i-1)*NN);      % roll angle (degrees)
    theta(i)       = 180/pi*uu(8+(i-1)*NN);      % pitch angle (degrees)
    psi(i)         = uu(9+(i-1)*NN);             % yaw angle (degrees)
    p(i)           = 180/pi*uu(10+(i-1)*NN);     % body angular rate along x-axis (degrees/s)
    q(i)           = 180/pi*uu(11+(i-1)*NN);     % body angular rate along y-axis (degrees/s)
    r(i)           = 180/pi*uu(12+(i-1)*NN);     % body angular rate along z-axis (degrees/s)
    Va(i)          = uu(13+(i-1)*NN);            % airspeed (m/s)
    alpha(i)       = 180/pi*uu(14+(i-1)*NN);     % angle of attack (degrees)
    beta(i)        = 180/pi*uu(15+(i-1)*NN);     % side slip angle (degrees)
    wn(i)          = uu(16+(i-1)*NN);            % wind in the North direction
    we(i)          = uu(17+(i-1)*NN);            % wind in the East direction
    wd(i)          = uu(18+(i-1)*NN);            % wind in the Down direction
    pn_c(i)        = uu(19+(i-1)*NN);            % commanded North position (meters)
    pe_c(i)        = uu(20+(i-1)*NN);            % commanded East position (meters)
    h_c(i)         = uu(21+(i-1)*NN);            % commanded altitude (meters)
    Va_c(i)        = uu(22+(i-1)*NN);            % commanded airspeed (meters/s)
    alpha_c(i)     = 180/pi*uu(23+(i-1)*NN);     % commanded angle of attack (degrees)
    beta_c(i)      = 180/pi*uu(24+(i-1)*NN);     % commanded side slip angle (degrees)
    phi_c(i)       = 180/pi*uu(25+(i-1)*NN);     % commanded roll angle (degrees)
    theta_c(i)     = 180/pi*uu(26+(i-1)*NN);     % commanded pitch angle (degrees)
    chi_c(i)       = 180/pi*uu(27+(i-1)*NN);     % commanded course (degrees)
    %     e_c(1)      = uu(25);
    %     e_c(2)      = uu(26);
    %     e_c(3)      = uu(27);
    %     e_c(4)      = uu(28);
    p_c(i)         = 180/pi*uu(28+isquat+(i-1)*NN);     % commanded body angular rate along x-axis (degrees/s)
    q_c(i)         = 180/pi*uu(29+isquat+(i-1)*NN);     % commanded body angular rate along y-axis (degrees/s)
    r_c(i)         = 180/pi*uu(30+isquat+(i-1)*NN);     % commanded body angular rate along z-axis (degrees/s)
    pn_hat(i)      = uu(31+isquat+(i-1)*NN);            % estimated North position (meters)
    pe_hat(i)      = uu(32+isquat+(i-1)*NN);            % estimated East position (meters)
    h_hat(i)       = uu(33+isquat+(i-1)*NN);            % estimated altitude (meters)
    Va_hat(i)      = uu(34+isquat+(i-1)*NN);            % estimated airspeed (meters/s)
    alpha_hat(i)   = 180/pi*uu(35+isquat+(i-1)*NN);     % estimated angle of attack (degrees)
    beta_hat(i)    = 180/pi*uu(36+isquat+(i-1)*NN);     % estimated side slip angle (degrees)
    phi_hat(i)     = 180/pi*uu(37+isquat+(i-1)*NN);     % estimated roll angle (degrees)
    theta_hat(i)   = 180/pi*uu(38+isquat+(i-1)*NN);     % estimated pitch angle (degrees)
    chi_hat(i)     = 180/pi*uu(39+isquat+(i-1)*NN);     % estimated course (degrees)
    p_hat(i)       = 180/pi*uu(40+isquat+(i-1)*NN);     % estimated body angular rate along x-axis (degrees/s)
    q_hat(i)       = 180/pi*uu(41+isquat+(i-1)*NN);     % estimated body angular rate along y-axis (degrees/s)
    r_hat(i)       = 180/pi*uu(42+isquat+(i-1)*NN);     % estimated body angular rate along z-axis (degrees/s)
    %    Vg_hat(i)      = uu(43+isquat+(i-1)*NN);            % estimated groundspeed
    %    wn_hat(i)      = uu(44+isquat+(i-1)*NN);            % estimated North wind
    %    we_hat(i)      = uu(45+isquat+(i-1)*NN);            % estimated East wind
    %    psi_hat(i)     = 180/pi*uu(46+isquat+(i-1)*NN);     % estimated heading
    %    bx_hat(i)      = uu(47+isquat+(i-1)*NN);            % estimated x-gyro bias
    %    by_hat(i)      = uu(48+isquat+(i-1)*NN);            % estimated y-gyro bias
    %    bz_hat(i)      = uu(49+isquat+(i-1)*NN);            % estimated z-gyro bias
    delta_e(i)     = 180/pi*uu(50+isquat+(i-1)*NN);     % elevator angle (degrees)
    delta_a(i)     = 180/pi*uu(51+isquat+(i-1)*NN);     % aileron angle (degrees)
    delta_r(i)     = 180/pi*uu(52+isquat+(i-1)*NN);     % rudder angle (degrees)
    delta_t(i)     = uu(53+isquat+(i-1)*NN);            % throttle setting (unitless)
end
t           = uu(end);

% Convert to euler angles
%[phi_c,theta_c,chi_c] = quat2euler(e_c);

% compute course angle
chi = 180/pi*atan2(Va.*sin(psi)+we, Va.*cos(psi)+wn);

% define persistent variables
persistent pn_handle
persistent pe_handle
persistent h_handle
persistent Va_handle
persistent alpha_handle
persistent beta_handle
persistent phi_handle
persistent theta_handle
persistent chi_handle
persistent p_handle
persistent q_handle
persistent r_handle
persistent delta_e_handle
persistent delta_a_handle
persistent delta_r_handle
persistent delta_t_handle


% first time function is called, initialize plot and persistent vars
numvars = 11;
if t==0
    figure(2), clf
    for i = 1:N
        
        subplot(numvars,N,1+N*(1-1) + (i-1))
        hold on
        pn_handle{i} = graph_y_yhat_yd(t, pn(i), pn_hat(i), pn_c(i), 'p_n', []);
        
        subplot(numvars,N,1+N*(2-1) + (i-1))
        hold on
        pe_handle{i} = graph_y_yhat_yd(t, pe(i), pe_hat(i), pe_c(i), 'p_e', []);
        
        subplot(numvars,N,1+N*(3-1) + (i-1))
        hold on
        h_handle{i} = graph_y_yhat_yd(t, h(i), h_hat(i), h_c(i), 'h', []);
        
        subplot(numvars,N,1+N*(4-1) + (i-1))
        hold on
        Va_handle{i} = graph_y_yhat_yd(t, Va(i), Va_hat(i), Va_c(i), 'V_a', []);
        
        subplot(numvars,N,1+N*(5-1) + (i-1))
        hold on
        phi_handle{i} = graph_y_yhat_yd(t, phi(i), phi_hat(i), phi_c(i), '\phi', []);

        subplot(numvars,N,1+N*(6-1) + (i-1))
        hold on
        theta_handle{i} = graph_y_yhat_yd(t, theta(i), theta_hat(i), theta_c(i), '\theta', []);

        subplot(numvars,N,1+N*(7-1) + (i-1))
        hold on
        chi_handle{i} = graph_y_yhat_yd(t, chi(i), chi_hat(i), chi_c(i), '\chi', []);
        
        subplot(numvars,N,1+N*(8-1) + (i-1))
        hold on
        delta_e_handle{i} = graph_y(t, delta_e(i), [], 'b');
        ylabel('\delta_e')

        subplot(numvars,N,1+N*(9-1) + (i-1))
        hold on
        delta_a_handle{i} = graph_y(t, delta_a(i), [], 'b');
        ylabel('\delta_a')

        subplot(numvars,N,1+N*(10-1) + (i-1))
        hold on
        delta_r_handle{i} = graph_y(t, delta_r(i), [], 'b');
        ylabel('\delta_r')

        subplot(numvars,N,1+N*(11-1) + (i-1))
        hold on
        delta_t_handle{i} = graph_y(t, delta_t(i), [], 'b');
        ylabel('\delta_t')
        
        %
        %
        %         subplot(8,2,4)
        %         hold on
        %         alpha_handle = graph_y_yhat_yd(t, alpha, alpha_hat, alpha_c, '\alpha', []);
        %
        %
        %
        %         subplot(8,2,6)
        %         hold on
        %         beta_handle = graph_y_yhat_yd(t, beta, beta_hat, beta_c, '\beta', []);
        %
        
        %
        %         subplot(8,2,8)
        %         hold on
        %         p_handle = graph_y_yhat_yd(t, p, p_hat, p_c, 'p', []);
        %
        
        %
        %         subplot(8,2,10)
        %         hold on
        %         q_handle = graph_y_yhat_yd(t, q, q_hat, q_c, 'q', []);
        %
        
        %
        %         subplot(8,2,12)
        %         hold on
        %         r_handle = graph_y_yhat_yd(t, r, r_hat, r_c, 'r', []);
        %
        
    end
    
    % at every other time step, redraw state variables
else
    for i = 1:N
        graph_y_yhat_yd(t, pn(i), pn_hat(i), pn_c(i), 'p_n', pn_handle{i});
        graph_y_yhat_yd(t, pe(i), pe_hat(i), pe_c(i), 'p_e', pe_handle{i});
        graph_y_yhat_yd(t, h(i), h_hat(i), h_c(i), 'h', h_handle{i});
        graph_y_yhat_yd(t, Va(i), Va_hat(i), Va_c(i), 'V_a', Va_handle{i});
        graph_y_yhat_yd(t, phi(i), phi_hat(i), phi_c(i), '\phi', phi_handle{i});
        graph_y_yhat_yd(t, theta(i), theta_hat(i), theta_c(i), '\theta', theta_handle{i});
        graph_y_yhat_yd(t, chi(i), chi_hat(i), chi_c(i), '\chi', chi_handle{i});
        
        graph_y(t, delta_e(i), delta_e_handle{i});
        graph_y(t, delta_a(i), delta_a_handle{i});
        graph_y(t, delta_r(i), delta_r_handle{i});
        graph_y(t, delta_t(i), delta_t_handle{i});
        
        %        graph_y_yhat_yd(t, alpha, alpha_hat, alpha_c, '\alpha', alpha_handle);
        %        graph_y_yhat_yd(t, beta, beta_hat, beta_c, '\beta', beta_handle);
        %        graph_y_yhat_yd(t, p, p_hat, p_c, 'p', p_handle);
        %        graph_y_yhat_yd(t, q, q_hat, q_c, 'q', q_handle);
        %        graph_y_yhat_yd(t, r, r_hat, r_c, 'r', r_handle);
        
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y with lable mylabel
function handle = graph_y(t, y, handle, color)

if isempty(handle),
    handle    = plot(t,y,color);
else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    %drawnow
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with lable mylabel
function handle = graph_y_yd(t, y, yd, lab, handle)

if isempty(handle),
    handle(1)    = plot(t,y,'b');
    handle(2)    = plot(t,yd,'g--');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its
% desired value yd in red, lab is the label on the graph
function handle = graph_y_yhat_yd(t, y, yhat, yd, lab, handle)

if isempty(handle),
    handle(1)   = plot(t,y,'b');
    handle(2)   = plot(t,yhat,'g--');
    handle(3)   = plot(t,yd,'r-.');
    ylabel(lab)
    set(get(gca,'YLabel'),'Rotation',0.0);
else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
    set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
    set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);
    %drawnow
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its
% desired value yd in red, lab is the label on the graph
function handle = graph_yl_yf(t, y_lead, y_follow, lab, handle)

if isempty(handle)
    handle(1)    = plot(t,y_lead,'b');
    handle(2)    = plot(t,y_follow,'r--');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y_lead]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),y_follow]);
    %drawnow
end

%
%=============================================================================
% sat
% saturates the input between high and low
%=============================================================================
%
function out=sat(in, low, high)

if in < low,
    out = low;
elseif in > high,
    out = high;
else
    out = in;
end

% end sat


