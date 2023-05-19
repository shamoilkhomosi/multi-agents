% Open the desired folder within the /results/webots directory and then 
% press run section in this script to generate an L2 norm graph

close all; clear; clc; 
controllers = ["EST_IDEAL","EST_CONSTANT", "EST_LINEAR","EST_QUAD","EST_ALL"];
disp = ["Ideal", "Constant", "Linear", "Quadratic", "RSSI"];
num_agents = 40;
num_leaders = ' 5';
nl = str2double(num_leaders);
leader_index = ceil(num_agents/nl/2);
lgain = 1;
ggain = 100;

figure('units','normalized','outerposition',[0 0 1 1]); set(gcf,'DefaultLineLineWidth',3); 
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');
for i = 1:length(controllers)
    val_y = movmean(get_norm(controllers(i), num_agents, nl, leader_index),5);
    semilogy(val_y, 'DisplayName', disp(i));
    hold on; 
end
legend('Location','northeast'); set(gca, 'FontSize', 35)
ideal = get_norm(controllers(1), num_agents, nl, leader_index);
settle_thres = 0.1*ideal(1);
ylim([0.06 11]);
xlim([0 700]); xlabel('Simulation Step [$$k$$]','interpreter','latex'); ylabel('$$L_2$$ Norm [m]','interpreter','latex');  
title(strcat('Convergence performance with',(num_leaders),' leaders ($$N=',num2str(num_agents),',\kappa=',num2str(lgain),',\nu=',num2str(ggain),')$$'),'interpreter','latex')
semilogy([0 10^6], [settle_thres settle_thres], 'k--', 'DisplayName', '5\% Settling Region')
hold off; grid on; box on;

legend;
saveas(gcf,strcat("l2norm",num_leaders,num2str(lgain),num2str(ggain),".png"))

%%
csv_data = readtable(strcat("sim_data_",strcat(controllers(5),".csv")));
csv_data = sortrows(csv_data);
csv_data = table2array(csv_data);
csv_data(any(csv_data==-1,2),:) = [];
block_size = sum(csv_data(:,1)==1);
time = [];


for i = 1:num_agents
    agent_indices = find(csv_data(:,1)==(i-1));
    [~, i_sort] = sort(abs(csv_data(agent_indices, 2)), 'descend');
    if (i>num_agents/2)
        plot3(1:length(agent_indices), csv_data(agent_indices, 1), sort(abs(csv_data(agent_indices, 2)), 'descend'), 'k-'); hold on
    else
        plot3(1:length(agent_indices), csv_data(agent_indices, 1), sort(-abs(csv_data(agent_indices, 2)), 'ascend'), 'k-'); hold on;
    end
    
end

%%
function e_norm = get_norm(wbts_contoller, num_agents, nl, leader_index)
    csv_data = readtable(strcat("sim_data_",strcat(wbts_contoller,".csv")));
    base_len = length(find(csv_data.ID==1));

    for i = 1:num_agents
        id_indices = find(csv_data.ID==i);
        if ~isempty(id_indices) 
            z(i,1:length(id_indices)) = table2array(csv_data(id_indices, 2))';
        end
    end
%     figure; surf(z)
    e_norm = sqrt(sum(z.^2,1));
end