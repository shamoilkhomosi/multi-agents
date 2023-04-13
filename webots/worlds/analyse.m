close; clear; clc;
controllers = ["EST_IDEAL","EST_CONSTANT","EST_LINEAR", "EST_ALL"];
% disp = ["Ideal", "Constant", "Linear", "RSSI"];

% t = 1:size(z,2);
% figure;
% surf(z, 'edgecolor', 'none')

figure; hold on; grid on; set(gcf,'DefaultLineLineWidth',1.3);
ideal = get_norm(controllers(1));
settle_thres = 0.1*ideal(1);
plot([0 1000], [settle_thres settle_thres], 'k--', 'DisplayName', '5% Settling Region')
ylim([0 8]);
xlim([0 3600]); xlabel('Time (t)'); ylabel('L_2 Norm'); legend;  
for i = 1:length(controllers)
    plot(movmean(get_norm(controllers(i)),3), 'DisplayName', controllers(i), 'LineWidth',1.5);
end
legend;

%%

function e_norm = get_norm(wbts_contoller)
    num_agents = 50;
    csv_data = readtable(strcat("sim_data_",strcat(wbts_contoller,".csv")));

    for i = 1:num_agents
        id_indices = find(csv_data.ID==i);
        z(i,1:length(id_indices)) = table2array(csv_data(id_indices, 2))';
    end

    e_norm = sqrt(sum(z.^2,1));
end