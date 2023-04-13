function dzdt = maspde5(t, z, x, phiL, phiR, Na, h, Nh, k, a, v, kr, r)

xsect = zeros(Na, Nh/Na);
for i=1:Na
    xsect(i,:) = x((i-1)*Nh/Na+1:(i)*Nh/Na);
end
z_soh = z;

dzdt = zeros(Nh, 1);
% define the boundary conditions
z(1) = phiL*(1-exp(-5*t));
z(Nh+1) = phiR*(1-exp(-5*t));
% dzdx   = zeros(Nh, 1);
% d2zdx2 = zeros(Nh, 1);

for i=1:Na
    i_global = (i-1)*Nh/Na+1:(i)*Nh/Na;
    x_lead = round(xsect(i,round(Nh/Na/2))/h)+1;
    z_lead = z(x_lead);
    
    if i > 2
        x_lead_ll = round(xsect(i-2,round(Nh/Na/2))/h)+1;
        z_lead_ll = z(x_lead_ll);
    else 
        x_lead_ll = 1;  % + 1 to have distinct X values for interp
        z_lead_ll = z(x_lead_ll);
    end

    if i > 1
        x_lead_l = round(xsect(i-1,round(Nh/Na/2))/h)+1;
        z_lead_l = z(x_lead_l);
    else
        x_lead_l = 1;
        z_lead_l = z(1);
    end

    if i < Na-1
        x_lead_rr = round(xsect(i+2,round(Nh/Na/2))/h)+1;
        z_lead_rr = z(x_lead_rr);
    else
        x_lead_rr = Nh+1;  % - 2 to have distinct X values for interp
        z_lead_rr = z(x_lead_rr);
    end
    
    if i < Na
        x_lead_r = round(xsect(i+1,round(Nh/Na/2))/h)+1;
        z_lead_r = z(x_lead_r);
    else
        x_lead_r = Nh+1;
        z_lead_r = z(x_lead_r);
    end
        
    for j=(i-1)*Nh/Na+1:x_lead
%         coeff = polyfit([x_lead_ll, x_lead_l, x_lead], [z_lead_ll, z_lead_l, z_lead], 2);
%         z_soh(j) = polyval(coeff, j);

        L0 = ((j-x_lead_l)*(j-x_lead_ll))/((x_lead-x_lead_l)*(x_lead-x_lead_ll))*z_lead;
        L1 = ((j-x_lead)*(j-x_lead_ll))/((x_lead_l-x_lead)*(x_lead_l-x_lead_ll))*z_lead_l;
        L2 = ((j-x_lead)*(j-x_lead_l))/((x_lead_ll-x_lead)*(x_lead_ll-x_lead_l))*z_lead_ll;
        if isnan(L0); L0=0; end
        if isnan(L1); L1=0; end
        if isnan(L2); L2=0; end
        z_soh(j) = L0+L1+L2;
    end

    for j=x_lead+1:(i)*Nh/Na
%         coeff = polyfit([x_lead, x_lead_r, x_lead_rr], [z_lead, z_lead_r, z_lead_rr], 2);
%         z_soh(j) = polyval(coeff, j);

        L0 = ((j-x_lead_r)*(j-x_lead_rr))/((x_lead-x_lead_r)*(x_lead-x_lead_rr))*z_lead;
        L1 = ((j-x_lead)*(j-x_lead_rr))/((x_lead_r-x_lead)*(x_lead_r-x_lead_rr))*z_lead_r;
        L2 = ((j-x_lead)*(j-x_lead_r))/((x_lead_rr-x_lead)*(x_lead_rr-x_lead_r))*z_lead_rr;
        if isnan(L0); L0=0; end
        if isnan(L1); L1=0; end
        if isnan(L2); L2=0; end
        z_soh(j) = L0+L1+L2;
    end
%     dzdt(i_global)   = a.*(z_soh( (i-1)*Nh/Na+1:(i)*Nh/Na )-z(i_global));
%     dzdt(x_lead)     = mu*(0-z_lead);

%     for j = i_global
%         if j == 1
%             dzdt(1) = 0;
%         elseif j == x_lead
%             dzdt(x_lead) = (z(j+1) - 2*z(j) + z(j-1))/h^2 + mu*(0 - z_lead);
%         elseif j < x_lead
%             dzdt(j) = (z(j+1) - 2*z(j) + z(j-1))/h^2 + a.*((z(j+1)-z(j)) - ((z_soh(j+1)-z_soh(j))));
%         elseif j > x_lead
%             dzdt(j) = (z(j+1) - 2*z(j) + z(j-1))/h^2 + a.*((z(j-1)-z(j)) - ((z_soh(j-1)-z_soh(j))));
%         end
%     end

    for j = i_global
        if j == 1
            dzdt(1) = v*(z(j+1) - z(j))/h  + a*z(j) - k*(z_soh(j)) + kr*r(j);
        elseif j == i_global(end)
            dzdt(j) = v*(-z(j-1) + z(j))/h + a*z(j) - k*(z(j)) + kr*r(j) ;
        elseif j == x_lead
            dzdt(j) = v*(z(j+1) - 2*z(j) + z(j-1))/h^2 + a*z_lead - k*(z_lead) + kr*r(j);
        else
            dzdt(j) = v*(z(j+1) - 2*z(j) + z(j-1))/h^2 + a*z(j) - k*(z_soh(j)) + kr*r(j);
        end
    end
end

end

















