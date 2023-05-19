function dzdt = maspde_rssi(t, z, x, phiL, phiR, Na, h, Nh, k, a, v, kr, r)

xsect = zeros(Na, Nh/Na);
for i=1:Na
    xsect(i,:) = x((i-1)*Nh/Na+1:(i)*Nh/Na);
end

dzdt = zeros(Nh, 1);
% define the boundary conditions
z(1) = phiL;
z(Nh+1) = phiR;

for i=1:Na
    i_global = (i-1)*Nh/Na+1:(i)*Nh/Na;
    x_lead = round(xsect(i,round(Nh/Na/2))/h)+1;
    z_lead = z(x_lead);
    
    for j = i_global
        pow = 2;
        den = 0;
        for l = 1:Na
            x_lead_k = round(xsect(l,round(Nh/Na/2))/h)+1;
            den = den + (1/abs(j-x_lead_k)^pow);
        end
    
        z_rssi = 0;
        for l = 1:Na
            x_lead_k = round(xsect(l,round(Nh/Na/2))/h)+1;
            lamda = (1/abs(j-x_lead_k)^pow) / den;
            z_rssi = z_rssi + lamda * z(x_lead_k);
        end
        
        % new
%         pow = 1;
%         l_dist_thres = (Nh/Na * 1.1);
%         l_dists = size(Na);
%         l_z = size(Na);
%         for l = 1:Na
%             x_lead_k = round(xsect(l,round(Nh/Na/2))/h)+1;
%             l_z(l) = z(x_lead_k);
%             l_dists(l) = abs(x_lead_k - j);
%         end
%         l_z(l_dists>l_dist_thres) = [];
%         l_dists(l_dists>l_dist_thres) = [];
%         den = sum(1./(l_dists.^pow));
% 
%         z_rssi = 0;
%         for l = 1:length(l_dists)
%             lamda = (1/l_dists(l)^pow) / den;
%             z_rssi = z_rssi + lamda * l_z(l);
%         end
%         z_rssi = l_z(1) + (l_z(2) - l_z(1))/(l_dists(1) + l_dists(2))*(l_dist(1));
        %

        if j == 400
            hellooo = 1;
        end

        if j == 1
            dzdt(j) = v*(z(j+1) - z(j))/h  + a*z(j) - k*(z_rssi) + kr*r(j)- v*(r(j+1) - r(j))/h;
        elseif j == Nh
            dzdt(j) = v*(-z(j-1) + z(j))/h + a*z(j) - k*(z(j)) + kr*r(j) - v*(-r(j-1) + r(j))/h ;
        elseif j == x_lead
            dzdt(j) = v*(z(j+1) - 2*z(j) + z(j-1))/h^2 + a*z_lead - k*(z_lead) + kr*r(j)  - v*(r(j+1) - 2*r(j) + r(j-1))/h^2;
        else
            dzdt(j) = v*(z(j+1) - 2*z(j) + z(j-1))/h^2 + a*z(j) - k*(z_rssi) + kr*r(j)  - v*(r(j+1) - 2*r(j) + r(j-1))/h^2;
        end
    end
end

end















