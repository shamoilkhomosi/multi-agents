clc; 
clear;
close all;

tend = 1.5; % time scale
xend = 1; % distance between left and right agent
h    = xend/1000; % step-size in space
k    = tend/1000; % step-size in time
Nh   = xend/h; % number of steps in space§
Nk   = tend/k; % number of steps in time§
Na = Nh/200;
tspan= linspace(0,tend,Nk + 1);
x = linspace(0,xend,Nh);

mu   = 10; % global controller 
a    = 4; % system dynamics   mode = pi^2 pi^2/xend^2
v    = 0.016; % local controller  *xend^2 correct term for bigger space? how is this optimised? can we not set it really high/low to achieve faster response?
kr   = 0; % feedforward gain (try also integral action) (= mu-a to avoid needing integral action for zero-offset tracking)
% ki   = 0.002;
ki = 0;

% r    = 3*cos(x/xend*2*pi) ; %target curve
r = x.*0;
% r = 10*sin(x/xend*2*pi);
phiL = 0; % steady state position of left boundary agent (leader?)
phiR = 0; % steady state position of right boundary agent (leader?)

r_num_stab = v^2*k/h^2; % stability parameter - should be <0.5 always

% define initial conditions and solve ODEs at each point in space
% z0 = 10.*x.^2.*(xend-x);
% z0 = 100.*x.^3.*(xend-x);
% z0 = x.*0;
% z0 = ones(length(x), 1);
% z0 = cos(x/xend*2*pi);
z0 = 10*sin(x/xend*2*pi); 
% z0 = sqrt(1-(x-1).^2) + 1;

% % re-add boundary conditions
% z(:,1) = phiL*(1-exp(temp_tc*t));
% z(:,Nh+1) = phiR*(1-exp(temp_tc*t));

tic
    [~, z] = ode15s(@maspde_ideal, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
toc
% re-add boundary conditions
z(:,1) = phiL;
z(:,Nh+1) = phiR;
e_norm_z_ideal = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
slice_z_ideal = z(100, 1:end);
z_ideal = z;

tic
    [~, z] = ode15s(@maspde_rssi, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
toc
% re-add boundary conditions
z(:,1) = phiL;
z(:,Nh+1) = phiR;
e_norm_z_rssi = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
slice_z_rssi = z(100, 1:end);
z_rssi = z;

tic
    [~, z] = ode15s(@maspde_con, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
toc
% re-add boundary conditions
z(:,1) = phiL;
z(:,Nh+1) = phiR;
e_norm_z_zoh = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
slice_z_zoh = z(100, 1:end);
z_zoh = z;

tic
    [~, z] = ode15s(@maspde_lin, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
toc
% re-add boundary conditions
z(:,1) = phiL;
z(:,Nh+1) = phiR;
e_norm_z_foh = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
slice_z_foh = z(100, 1:end);
z_foh = z;

tic
    [~, z] = ode15s(@maspde5, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
toc
% re-add boundary conditions
z(:,1) = phiL;
z(:,Nh+1) = phiR;
e_norm_z_soh2 = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
slice_z_soh2 = z(100, 1:end);
z_soh2 = z;

tic
    [~, z] = ode15s(@maspde_qua, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
toc
% re-add boundary conditions
z(:,1) = phiL;
z(:,Nh+1) = phiR;
e_norm_z_soh3 = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
slice_z_soh3 = z(100, 1:end);
z_soh3 = z;

% PIs
settle_thres = 0.05*e_norm_z_ideal(1);
settle_time_ideal = find(e_norm_z_ideal <settle_thres, 1);
settle_time_zoh = find(e_norm_z_zoh <settle_thres, 1);
settle_time_foh = find(e_norm_z_foh <settle_thres, 1);
settle_time_soh2 = find(e_norm_z_soh2 <settle_thres, 1);
settle_time_soh3 = find(e_norm_z_soh3 <settle_thres, 1);
settle_time_rssi = find(e_norm_z_rssi <settle_thres, 1);

std_zoh = max(std(z_zoh-z_ideal));
std_foh = max(std(z_foh-z_ideal));
std_soh2 = max(std(z_soh2-z_ideal));
std_soh3 = max(std(z_soh3-z_ideal));
std_rssi = max(std(z_rssi-z_ideal));

% Plot figures

set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');

figure('units','normalized','outerposition',[0 0 1 1]); set(gcf,'DefaultLineLineWidth',2); semilogy(e_norm_z_zoh, 'r', 'DisplayName', 'Constant'); hold on; 
semilogy(e_norm_z_foh, 'b', 'DisplayName', 'Linear'); semilogy(e_norm_z_soh2, 'm', 'DisplayName', 'Quadratic'); semilogy(e_norm_z_soh3, 'g', 'DisplayName', 'Cubic (Convex)');
semilogy(e_norm_z_rssi, 'DisplayName', 'RSSI'); 
semilogy(e_norm_z_ideal, 'k-', 'DisplayName', 'Ideal'); 
plot([0 tend/k], [settle_thres settle_thres], 'k--', 'DisplayName', '5\% Settling Region')
xlim([0 1000]); xlabel('Time (t)','interpreter','latex'); ylabel('$$L_2$$ Norm','interpreter','latex'); legend;  
ylim([1 300]);
title('Performance of MAS with different piecewise state approximations','interpreter','latex')
set(gca, 'FontSize', 30)
legend('Location','southwest'); 
saveas(gcf,strcat("decay_norm.png"))

figure('units','normalized','outerposition',[0 0 1 1]); set(gcf,'DefaultLineLineWidth',2); 
box on;
set(gca, 'FontSize', 15)
subplot(3,2,1); sel=Nh; surf(z_ideal(1:round(tend/k/sel):end, 1:round(xend/h/sel):end), 'edgecolor', 'none', 'DisplayName', 'Ideal'); title('Ideal','interpreter','latex');
set(gca, 'FontSize', 15); ylim([0 400])
subplot(3,2,2); surf(z_zoh(1:round(tend/k/sel):end, 1:round(xend/h/sel):end), 'edgecolor', 'none', 'DisplayName', 'Constant'); title('Constant','interpreter','latex');
set(gca, 'FontSize', 15); ylim([0 400])
subplot(3,2,3); surf(z_foh(1:round(tend/k/sel):end, 1:round(xend/h/sel):end), 'edgecolor', 'none', 'DisplayName', 'Linear'); title('Linear','interpreter','latex');
set(gca, 'FontSize', 15); ylim([0 400])
subplot(3,2,4); surf(z_soh3(1:round(tend/k/sel):end, 1:round(xend/h/sel):end), 'edgecolor', 'none', 'DisplayName', 'Cubic (Convex)'); title('Cubic (Convex)','interpreter','latex');
set(gca, 'FontSize', 15); ylim([0 400])
subplot(3,2,5); surf(z_soh2(1:round(tend/k/sel):end, 1:round(xend/h/sel):end), 'edgecolor', 'none', 'DisplayName', 'Quadratic'); title('Quadratic','interpreter','latex');
set(gca, 'FontSize', 15); ylim([0 400])
subplot(3,2,6); surf(z_rssi(1:round(tend/k/sel):end, 1:round(xend/h/sel):end), 'edgecolor', 'none', 'DisplayName', 'RSSI'); title('RSSI','interpreter','latex');
set(gca, 'FontSize', 15); ylim([0 400])
ax = findobj(gcf,'Type','Axes');
% sgtitle('Performance of a MAS state feedback with different piecewise state approximations for the agents')
for i=1:length(ax)
    xlabel(ax(i),{'Agent Position (x)'},'interpreter','latex')
    ylabel(ax(i),{'Time (t)'},'interpreter','latex')
    zlabel(ax(i),{'Agent State (z)'},'interpreter','latex')
end
saveas(gcf,strcat("decay.png"))

figure('units','normalized','outerposition',[0 0 1 1]); set(gcf,'DefaultLineLineWidth',2); plot(slice_z_zoh, 'r', 'DisplayName', 'Constant'); hold on; plot(slice_z_foh, 'b', 'DisplayName', 'Linear'); plot(slice_z_soh2, 'm', 'DisplayName', 'Quadratic'); plot(slice_z_soh3, 'g', 'DisplayName', 'Cubic (Convex)'); 
plot(slice_z_rssi, 'DisplayName', 'RSSI'); 
% plot(slice_z_ideal, 'k-', 'DisplayName', 'Ideal');
plot([r 0], 'k-', 'DisplayName', 'Target Curve');
set(gca, 'FontSize', 30)
scatter(x(Nh/Na/2:Nh/Na:Nh)*Nh,z(100,Nh/Na/2:Nh/Na:Nh),1000, 'k', "*",'DisplayName','Leader agent','DefaultLineLineWidth',10)
xlim([0 Nh]); xlabel('Agent Position (x)','interpreter','latex'); ylabel('Agent State (z)','interpreter','latex'); legend; 
title('Transient formations with different piecewise state approximations','interpreter','latex')
set(gca, 'FontSize', 30)
saveas(gcf,strcat("decay_slice.png"))