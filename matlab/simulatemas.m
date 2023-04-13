clc; 
clear;
close all;
clear maspde_ideal;
% find successive unstabilising approximation controllers. fix v, 0, find a such that it stabilises the system
% then tune v
% try higher order approx for consensus - see taylor expansion of derivatives - residue small o of h reducing
% for global controllers

tend = 2; % time scale
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
v    = 0.008; % local controller  *xend^2 correct term for bigger space? how is this optimised? can we not set it really high/low to achieve faster response?
kr   = 0; % feedforward gain (try also integral action) (= mu-a to avoid needing integral action for zero-offset tracking)
% ki   = 0.002;
ki = 0;

% r    = 3*cos(x/xend*2*pi) ; %target curve
r = x.*0;
% r = 10*sin(x/xend*2*pi);
phiL = 0; % steady state position of left boundary agent (leader?)
phiR = 0; % steady state position of right boundary agent (leader?)

r_num_stab = v^2*k/h^2; % stability parameter - check always

% define initial conditions and solve ODEs at each point in space
% z0 = x.*(xend-x);
% z0 = 100.*x.^3.*(xend-x);
% z0 = x.*0;
% for n = 1:100
%     z0 = z0+ 4/pi.*(1/n*sin(n*pi*x/xend));
% end
% z0 = ones(length(x), 1);
% z0 = cos(x/xend*2*pi);
z0 = 10*sin(x/xend*2*pi); % try halfcycle
% z0 = sqrt(1-(x-1).^2) + 1;
% [t, z] = ode15s(@maspde1, tspan, z0, [], phiL, phiR, h, Nh, mu, a);
% % re-add (time-varying) boundary conditions
% z(:,1) = phiL*(1-exp(temp_tc*t));
% z(:,Nh+1) = phiR*(1-exp(temp_tc*t));

%
[t, z] = ode15s(@maspde_ideal, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
% re-add (time-varying) boundary conditions
z(:,1) = phiL;
z(:,Nh+1) = phiR;
e_norm_z_ideal = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
slice_z_ideal = z(100, 1:end);
z_ideal = z;

[t, z] = ode15s(@maspde_rssi, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
% re-add (time-varying) boundary conditions
z(:,1) = phiL;
z(:,Nh+1) = phiR;
e_norm_z_rssi = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
slice_z_rssi = z(100, 1:end);
z_rssi = z;

[t, z] = ode15s(@maspde_con, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
% re-add (time-varying) boundary conditions
z(:,1) = phiL;
z(:,Nh+1) = phiR;
e_norm_z_zoh = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
slice_z_zoh = z(100, 1:end);
z_zoh = z;

[t, z] = ode15s(@maspde_lin, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
% re-add (time-varying) boundary conditions
z(:,1) = phiL;
z(:,Nh+1) = phiR;
e_norm_z_foh = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
slice_z_foh = z(100, 1:end);
z_foh = z;

% [t, z] = ode15s(@maspde4, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a);
% % re-add (time-varying) boundary conditions
% z(:,1) = phiL*(1-exp(temp_tc*t));
% z(:,Nh+1) = phiR*(1-exp(temp_tc*t));
% e_norm_z_soh = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
% figure; sel=50; surf(z(1:round(tend/k/sel):end, 1:round(xend/h/sel):end), 'edgecolor', 'none')
% % figure; plot(z(5, 1:end), 'c', 'LineWidth', 2)
% 
[t, z] = ode15s(@maspde5, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
% re-add (time-varying) boundary conditions
z(:,1) = phiL;
z(:,Nh+1) = phiR;
e_norm_z_soh2 = sqrt(sum((z-repmat([r 0],Nk+1,1)).^2,2));
slice_z_soh2 = z(100, 1:end);
z_soh2 = z;

[t, z] = ode15s(@maspde_qua, tspan, z0, [], x, phiL, phiR, Na, h, Nh, mu, a, v, kr, r);
% re-add (time-varying) boundary conditions
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

figure; subplot(3,2,1); sel=Nh; surf(z_ideal(1:round(tend/k/sel):end, 1:round(xend/h/sel):end), 'edgecolor', 'none', 'DisplayName', 'Ideal'); title('Ideal');
subplot(3,2,2); surf(z_zoh(1:round(tend/k/sel):end, 1:round(xend/h/sel):end), 'edgecolor', 'none', 'DisplayName', 'Constant'); title('Constant');
subplot(3,2,3); surf(z_foh(1:round(tend/k/sel):end, 1:round(xend/h/sel):end), 'edgecolor', 'none', 'DisplayName', 'Linear'); title('Linear');
subplot(3,2,4); surf(z_soh3(1:round(tend/k/sel):end, 1:round(xend/h/sel):end), 'edgecolor', 'none', 'DisplayName', 'Cubic (Convex)'); title('Cubic (Convex)');
subplot(3,2,5); surf(z_soh2(1:round(tend/k/sel):end, 1:round(xend/h/sel):end), 'edgecolor', 'none', 'DisplayName', 'Quadratic'); title('Quadratic');
subplot(3,2,6); surf(z_rssi(1:round(tend/k/sel):end, 1:round(xend/h/sel):end), 'edgecolor', 'none', 'DisplayName', 'RSSI'); title('RSSI');
ax = findobj(gcf,'Type','Axes');
% sgtitle('Performance of a MAS state feedback with different piecewise state approximations for the agents')
for i=1:length(ax)
    xlabel(ax(i),{'Agent Position (x)'})
    ylabel(ax(i),{'Time (t)'})
    zlabel(ax(i),{'Agent State (z)'})
end

figure; set(gcf,'DefaultLineLineWidth',1); plot(slice_z_zoh, 'r', 'DisplayName', 'Constant'); hold on; plot(slice_z_foh, 'b', 'DisplayName', 'Linear'); plot(slice_z_soh2, 'm', 'DisplayName', 'Quadratic'); plot(slice_z_soh3, 'g', 'DisplayName', 'Cubic (Convex)'); 
plot(slice_z_rssi, 'k', 'DisplayName', 'RSSI'); plot(slice_z_ideal, 'k-', 'DisplayName', 'Ideal'); 
scatter(x(Nh/Na/2:Nh/Na:Nh)*Nh,z(100,Nh/Na/2:Nh/Na:Nh),100, 'k', "*",'DisplayName','Leader agent')
xlim([0 Nh]); xlabel('Agent Position (x)'); ylabel('Agent State (z)'); legend; 
% title('Comparision of the agents'' states at t=100ms with different piecewise state approximations')

figure; set(gcf,'DefaultLineLineWidth',1.3); semilogy(e_norm_z_zoh, 'r', 'DisplayName', 'Constant'); hold on; semilogy(e_norm_z_foh, 'b', 'DisplayName', 'Linear'); semilogy(e_norm_z_soh2, 'm', 'DisplayName', 'Quadratic'); semilogy(e_norm_z_soh3, 'g', 'DisplayName', 'Cubic (Convex)');
semilogy(e_norm_z_rssi, 'k', 'DisplayName', 'RSSI'); 
semilogy(e_norm_z_ideal, 'k-', 'DisplayName', 'Ideal'); 
plot([0 tend/k], [settle_thres settle_thres], 'k--', 'DisplayName', '5% Settling Region')
xlim([0 tend/k]); xlabel('Time (t)'); ylabel('L_2 Norm'); legend;  
% ylim([1 300]);
% title('Performance of a MAS state feedback with different piecewise state approximations for the agents')

%%




%%
% domain definition
Na = 5; % number of agents (make sure its divisible by Nh)
domains = [5 10 20 25 50 100 125 200 250 500 1000];
xsect = zeros(Na, Nh/Na);
for i=1:Na
    xsect(i,:) = x((i-1)*Nh/Na+1:(i)*Nh/Na);
end

% piecewise approximation (ZOH)
z_zoh = z;
for t=1:tend/k
    for i=1:Na
        for j=(i-1)*Nh/Na+1:(i)*Nh/Na
            z_zoh(t, j) = z(t,round(xsect(i,round(Nh/Na/2))/h)+1);
        end
    end
end

% piecewise approximation (FOH)
z_foh = z;
for t=1:tend/k
    for i=1:Na
        x_lead = round(xsect(i,round(Nh/Na/2))/h)+1;
        z_lead = z(t,x_lead);
        
        if i > 1
            x_lead_l = round(xsect(i-1,round(Nh/Na/2))/h)+1;
            z_lead_l = z(t,x_lead_l);
        else
            x_lead_l = 1;
            z_lead_l = z(t,1);
        end
          
        if i < Na
            x_lead_r = round(xsect(i+1,round(Nh/Na/2))/h)+1;
            z_lead_r = z(t,x_lead_r);
        else
            x_lead_r = Nh+1;
            z_lead_r = z(t,x_lead_r);
        end
        
        for j=(i-1)*Nh/Na+1:x_lead
            z_foh(t, j) = z_lead_l + (z_lead - z_lead_l)/(x_lead - x_lead_l)*(j - x_lead_l);
        end
        
        for j=x_lead+1:(i)*Nh/Na
            z_foh(t, j) = z_lead + (z_lead_r - z_lead)/(x_lead_r - x_lead)*(j - x_lead);
        end
    end
end
%%
% piecewise approximation (SOH)
z_soh = z;
for t=1:tend/k
    for i=1:Na
        x_lead = round(xsect(i,round(Nh/Na/2))/h)+1;
        z_lead = z(t,x_lead);
        
        if i > 2
            x_lead_ll = round(xsect(i-2,round(Nh/Na/2))/h)+1;
            z_lead_ll = z(t,x_lead_ll);
        else 
            x_lead_ll = 1;  % + 1 to have distinct X values for interp
            z_lead_ll = z(t,x_lead_ll);
        end
        
        if i > 1
            x_lead_l = round(xsect(i-1,round(Nh/Na/2))/h)+1;
            z_lead_l = z(t,x_lead_l);
        else
            x_lead_l = 1;
            z_lead_l = z(t,1);
        end
        
        if i < Na-1
            x_lead_rr = round(xsect(i+2,round(Nh/Na/2))/h)+1;
            z_lead_rr = z(t,x_lead_rr);
        else
            x_lead_rr = Nh+1;  % - 2 to have distinct X values for interp
            z_lead_rr = z(t,x_lead_rr);
        end
        
        if i < Na
            x_lead_r = round(xsect(i+1,round(Nh/Na/2))/h)+1;
            z_lead_r = z(t,x_lead_r);
        else
            x_lead_r = Nh+1;
            z_lead_r = z(t,x_lead_r);
        end
        
%         for j=(i-1)*Nh/Na+1:(i)*Nh/Na
%             coeff = polyfit([x_lead_ll, x_lead_l, x_lead, x_lead_r, x_lead_rr], [z_lead_ll, z_lead_l, z_lead, z_lead_r, z_lead_rr], 2);
%             z_soh(t, j) = polyval(coeff, j);
%         end 

%         for j=(i-1)*Nh/Na+1:(i)*Nh/Na
%             coeff = polyfit([x_lead_l, x_lead, x_lead_r], [z_lead_l, z_lead, z_lead_r], 2);
%             z_soh(t, j) = polyval(coeff, j);
%         end 

        for j=(i-1)*Nh/Na+1:x_lead
            coeff = polyfit([x_lead_ll, x_lead_l, x_lead], [z_lead_ll, z_lead_l, z_lead], 2);
            z_soh(t, j) = polyval(coeff, j);
        end
        
        for j=x_lead+1:(i)*Nh/Na
            coeff = polyfit([x_lead, x_lead_r, x_lead_rr], [z_lead, z_lead_r, z_lead_rr], 2);
            z_soh(t, j) = polyval(coeff, j);
        end
        
    end
end
%%

%plot
xaxis = linspace(0, xend, Nh+1);
yaxis = tspan;
imagesc(xaxis, yaxis, z);
grid on;
xlabel('axial position');
ylabel('timespan');
title('Location of Agents')
colorbar;
colormap jet;
%%
sel = 20;
figure;
surf(z(1:round(tend/k/sel):end, 1:round(xend/h/sel):end), 'edgecolor', 'none')
hold on;
surf(z_zoh(1:round(tend/k/sel):end-1, 1:round(xend/h/sel):end-1), 'edgecolor', 'none')
hold on;
surf(z_foh(1:round(tend/k/sel):end-1, 1:round(xend/h/sel):end-1), 'edgecolor', 'none')
hold on;
surf(z_soh(1:round(tend/k/sel):end-1, 1:round(xend/h/sel):end-1), 'edgecolor', 'none')

%%
t = 1;
type = {'zoh', 'foh', 'soh'};
e_norm = zeros(length(type), length(tspan));
figure;

plot(z(t, 1:end), 'k', 'LineWidth', 2)
hold on;
plot(z_zoh(t, 1:end-1), 'g')
e_norm(1,:) = sqrt(sum((z_zoh).^2, 2));
hold on;
plot(z_foh(t, 1:end-1), 'b')
e_norm(2,:) = sqrt(sum((z_foh).^2, 2));
hold on;
plot(z_soh(t, 1:end-1), 'r')
e_norm(3,:) = sqrt(sum((z_soh).^2, 2));
hold off;
legend('Target function', 'Zero-order piecewise', 'First-order piecewise', 'Second-order piecewise');
xlim([0 Nh]);


%%
% 
% %%
% % ZOH across domains
% z_zoh_domains = zeros(size(z, 1), size(z, 2), length(domains));
% for d=1:length(domains)
%     Na_domain = domains(d);
%     xsect_domains = zeros(Na_domain, Nh/Na_domain);
%     for i=1:Na_domain
%         xsect_domains(i,:) = x((i-1)*Nh/Na_domain+1:(i)*Nh/Na_domain);
%     end
%     for t=1:tend/k
%         for i=1:Na_domain
%             for j=(i-1)*Nh/Na_domain+1:(i)*Nh/Na_domain
%                 z_zoh_domains(t, j, d) = z(t,round(xsect_domains(i,round(Nh/Na_domain/2))/h)+1);
%             end
%         end
%     end
% end
% 
% %%
% % FOH across domains
% z_foh_domains = zeros(size(z, 1), size(z, 2), length(domains));
% for d=1:length(domains)
%     Na_domain = domains(d);
%     xsect_domains = zeros(Na_domain, Nh/Na_domain);
%     for i=1:Na_domain
%         xsect_domains(i,:) = x((i-1)*Nh/Na_domain+1:(i)*Nh/Na_domain);
%     end
%     for t=1:tend/k
%         for i=1:Na_domain
%             x_lead = round(xsect_domains(i,round(Nh/Na_domain/2))/h)+1;
%             z_lead = z(t,x_lead);
% 
%             if i > 1
%                 x_lead_l = round(xsect_domains(i-1,round(Nh/Na_domain/2))/h)+1;
%                 z_lead_l = z(t,x_lead_l);
%             else
%                 x_lead_l = 1;
%                 z_lead_l = z(t,1);
%             end
% 
%             if i < Na_domain
%                 x_lead_r = round(xsect_domains(i+1,round(Nh/Na_domain/2))/h)+1;
%                 z_lead_r = z(t,x_lead_r);
%             else
%                 x_lead_r = Nh+1;
%                 z_lead_r = z(t,x_lead_r);
%             end
% 
%             for j=(i-1)*Nh/Na_domain+1:x_lead
%                 z_foh_domains(t, j, d) = z_lead_l + (z_lead - z_lead_l)/(x_lead - x_lead_l)*(j - x_lead_l);
%             end
% 
%             for j=x_lead+1:(i)*Nh/Na_domain
%                 z_foh_domains(t, j, d) = z_lead + (z_lead_r - z_lead)/(x_lead_r - x_lead)*(j - x_lead);
%             end
%         end
%     end
% end
% %%
% 
% rmse_zoh_domains = zeros(length(domains),1);
% for i=1:length(domains)
%     rmse_zoh_domains(i) = sqrt(sum((z-z_zoh_domains(:,:,i)).^2, 'all')/sum(size((z))));
% end
% 
% rmse_foh_domains = zeros(length(domains),1);
% for i=1:length(domains)
%     rmse_foh_domains(i) = sqrt(sum((z-z_foh_domains(:,:,i)).^2, 'all')/sum(size((z))));
% end
% figure;
% plot(domains, rmse_zoh_domains, domains, rmse_foh_domains, 'LineWidth', 2);

%%
% defining boundary conditions in the real world
% stability (r) calculation when a is non-zero
% does using forward, center, right FDA on left, interior, right extremes
% of axial position give more accurate solution?
% are phiL and phiR positions of each leader-pair? 
% expanding to complex z functions (by varying a?)

% can we first set z as the target function and then solve for the
% parameters of the heat diff eqn to find out commands for the agents?

% inreased connectuvity +> better performance?
% 
% lit review
% mat program to solve pde
% mat program with ontrol inout piecewise and linear
% compare
% increased connectivity
% compare with prev two
% 
% hardware implementation
% 
% shubham + gross 

%%
num_leaders = 7; 
cluster_size = 5;
num_agents = num_leaders*cluster_size;

leader_indices = zeros(num_leaders, 1);
for i = 1:num_leaders
    leader_indices(i) = (i-1)*cluster_size + ceil(cluster_size/2);
end

A_global = zeros(num_agents); %% RSSI
for i = 1:num_agents
    for j = 1:num_agents
        if (ismember([i j], leader_indices) & (i==j))
            A_global(i,j) = 1;
        end
        
        if (ismember(j, leader_indices) && ~ismember(i, leader_indices))
            num = 1/abs(i-j).^1;
            den = sum(1./abs(i-leader_indices).^1);
            A_global(i,j) = num/den;
        end
    end
end

% A_global = eye(num_agents); %% IDEAL

% A_global = zeros(num_agents); %% CONSTANT
% for i = 1:num_agents 
%     for l = 1:length(leader_indices)
%         j = leader_indices(l);
%         if(i>(j-cluster_size/2) && i<(j+cluster_size/2))
%             A_global(i,j) = 1;
%         end
%     end
% end

A_local = zeros(num_agents);
for i = 1:num_agents
    for j = 1:num_agents
        if (i == j)
            A_local(i,j) = -2;
        end

        if (i == j) && (i == 1 || i == num_agents)
            A_local(i,j) = -1;
        end  

        if (abs(i-j) == 1)
            A_local(i,j) = 1;
        end
    end
end

A_dynamics = eye(num_agents).*0;

A = (0.5/0.01).*A_local - 20.*A_global + A_dynamics;
poles = eig(A);
mean(poles)

close; figure;
plot(real(poles), imag(poles), 'ro')
grid on; hold on;

% A_ss = A_dynamics;
% B_ss = A_global - A_local;
% C_ss = zeros(num_leaders, num_agents);
% for l = 1:num_leaders
%     for a = 1:num_agents
%         if (ismember(a, leader_indices) && l==find(leader_indices==a))
%             C_ss(l,a) = 1;
%         else
%             C_ss(l,a) = 0;
%         end
%     end
% end
% D_ss = zeros(num_leaders, num_agents);
% model = ss(A_ss-B_ss, B_ss, C_ss, D_ss);
% figure; step(model)