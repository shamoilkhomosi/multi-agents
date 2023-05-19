% This script is used to generate the settling times when different number
% of leaders are present in the MAS. The iterations for the number of the
% leaders is defined in 'Nas' variable.

clc; clear; close;

tend = 3; % time scale
xend = 1; % distance between left and right agent
h    = xend/1000; % step-size in space
k    = tend/1000; % step-size in time
Nh   = xend/h; % number of steps in space§
Nk   = tend/k; % number of steps in time§
Nas = [Nh/5 Nh/8 Nh/10 Nh/20 Nh/25 Nh/40 Nh/50 Nh/100 Nh/125 Nh/200];
tspan= linspace(0,tend,Nk + 1);
x = linspace(0,xend,Nh);

mu   = 10; % global controller 
a    = 4; % system dynamics   mode = pi^2 pi^2/xend^2
v    = 0.016; % local controller  *xend^2 correct term for bigger space? how is this optimised? can we not set it really high/low to achieve faster response?
kr   = 0; % feedforward gain (try also integral action) (= mu-a to avoid needing integral action for zero-offset tracking)

% r    = 3*cos(x/xend*2*pi) ; %target curve
r = x.*0;
% r = 10*sin(x/xend*2*pi);
phiL = 0; % steady state position of left boundary agent (leader?)
phiR = 0; % steady state position of right boundary agent (leader?)

r_num_stab = v^2*k/h^2; % stability parameter 

% define initial conditions and solve ODEs at each point in space
% z0 = 10.*x.*(xend-x);
% z0 = 10.*x.^2.*(xend-x);
% z0 = x.*(xend-x);
% z0 = 100.*x.^3.*(xend-x);
% z0 = x.*0;
% z0 = ones(length(x), 1);
% z0 = cos(x/xend*2*pi);
z0 = 10*sin(x/xend*2*pi);
% z0 = sqrt(1-(x-1).^2) + 1;

for Na = Nas
    %
    [t, z] = ode15s(@maspde_ideal, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
    % re-add (time-varying) boundary conditions
    z(:,1) = phiL;
    z(:,Nh+1) = phiR;
    e_norm_z_ideal = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
    z_ideal = z;
    
    [t, z] = ode15s(@maspde_rssi, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
    % re-add (time-varying) boundary conditions
    z(:,1) = phiL;
    z(:,Nh+1) = phiR;
    e_norm_z_rssi = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
    z_rssi = z;
    
    [t, z] = ode15s(@maspde_con, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
    % re-add (time-varying) boundary conditions
    z(:,1) = phiL;
    z(:,Nh+1) = phiR;
    e_norm_z_zoh = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
    z_zoh = z;
    
    [t, z] = ode15s(@maspde_lin, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
    % re-add (time-varying) boundary conditions
    z(:,1) = phiL;
    z(:,Nh+1) = phiR;
    e_norm_z_foh = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
    z_foh = z;

    [t, z] = ode15s(@maspde5, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
    % re-add (time-varying) boundary conditions
    z(:,1) = phiL;
    z(:,Nh+1) = phiR;
    e_norm_z_soh2 = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
    z_soh2 = z;
    
    [t, z] = ode15s(@maspde_qua, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
    % re-add (time-varying) boundary conditions
    z(:,1) = phiL;
    z(:,Nh+1) = phiR;
    e_norm_z_soh3 = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
    z_soh3 = z;

    e_norm_z_ideal = [e_norm_z_ideal' 0];
    e_norm_z_zoh = [e_norm_z_zoh' 0];
    e_norm_z_foh = [e_norm_z_foh' 0];
    e_norm_z_soh2 = [e_norm_z_soh2' 0];
    e_norm_z_soh3 = [e_norm_z_soh3' 0];
    e_norm_z_rssi = [e_norm_z_rssi' 0];
    
    % PIs
    settle_thres = 0.05*e_norm_z_ideal(1);
    settle_time_ideal(find(Nas==Na)) = find(e_norm_z_ideal <settle_thres, 1);
    settle_time_zoh(find(Nas==Na)) = find(e_norm_z_zoh <settle_thres, 1);
    settle_time_foh(find(Nas==Na)) = find(e_norm_z_foh <settle_thres, 1);
    settle_time_soh2(find(Nas==Na)) = find(e_norm_z_soh2 <settle_thres, 1);
    settle_time_soh3(find(Nas==Na)) = find(e_norm_z_soh3 <settle_thres, 1);
    settle_time_rssi(find(Nas==Na)) = find(e_norm_z_rssi <settle_thres, 1);
end

save("num_leaders_4.mat", "settle_time_ideal", "settle_time_zoh", "settle_time_foh","settle_time_soh2", "settle_time_soh3", "settle_time_rssi")
%%

% Plot the results

Nh   = 1000; % number of steps in space
Nas = [Nh/5 Nh/8 Nh/10 Nh/20 Nh/25 Nh/40 Nh/50 Nh/100 Nh/125 Nh/200];

load("num_leaders.mat")
 figure; grid on; hold on; legend; box on;
set(gca, 'FontSize', 20); legend('Location','northeast'); 
xlim([3 42]);
ylabel('Settling time [t]', 'interpreter', 'latex')
xlabel('Number of leaders', 'interpreter', 'latex')
title('Effect of increasing the number of leaders on settling time ($$ N = 1000$$)', 'interpreter', 'latex', 'FontSize', 20)
width = 1.9;
% settle_time_soh3 = settle_time_soh2-0.2;
% settle_time_ideal = settle_time_ideal-0.4;
semilogy(Nas, settle_time_ideal.*2.0,'Marker','square','LineWidth',width, 'DisplayName','Ideal');
semilogy(Nas, settle_time_zoh.*2.0,'Marker','square','LineWidth',width, 'DisplayName','Constant');
semilogy(Nas, settle_time_foh.*2.0,'Marker','square','LineWidth',width, 'DisplayName','Linear');
semilogy(Nas, settle_time_soh2.*2.0,'Marker','square','LineWidth',width, 'DisplayName','Quadratic');
semilogy(Nas, settle_time_soh3.*2.0,'Marker','square','LineWidth',width, 'DisplayName','Cubic');
semilogy(Nas, settle_time_rssi.*2.0,'Marker','square','LineWidth',width, 'DisplayName','RSSI');