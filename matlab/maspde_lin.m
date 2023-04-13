function dzdt = maspde_lin(t, z, x, phiL, phiR, Na, h, Nh, k, a, v, kr, r)

xsect = zeros(Na, Nh/Na);
for i=1:Na
    xsect(i,:) = x((i-1)*Nh/Na+1:(i)*Nh/Na);
end
z_foh = z;

dzdt = zeros(Nh, 1);
% define the boundary conditions
z(1) = phiL;
z(Nh+1) = phiR;
% dzdx   = zeros(Nh, 1);
% d2zdx2 = zeros(Nh, 1);

for i=1:Na
    i_global = (i-1)*Nh/Na+1:(i)*Nh/Na;
    x_lead = round(xsect(i,round(Nh/Na/2))/h)+1;
    z_lead = z(x_lead);
    
    if i > 1
        x_lead_l = round(xsect(i-1,round(Nh/Na/2))/h)+1;
        z_lead_l = z(x_lead_l);
    else
        x_lead_l = 1;
        z_lead_l = z(1);
    end

    if i < Na
        x_lead_r = round(xsect(i+1,round(Nh/Na/2))/h)+1;
        z_lead_r = z(x_lead_r);
    else
        x_lead_r = Nh+1;
        z_lead_r = z(x_lead_r);
    end

    for j=(i-1)*Nh/Na+1:x_lead
        z_foh(j) = z_lead_l + (z_lead - z_lead_l)/(x_lead - x_lead_l)*(j - x_lead_l);
    end

    for j=x_lead+1:(i)*Nh/Na
        z_foh(j) = z_lead + (z_lead_r - z_lead)/(x_lead_r - x_lead)*(j - x_lead);
    end

%     for j=(i-1)*Nh/Na+1:(i)*Nh/Na
%         z_foh(j) = z_lead + (z_lead_r - z_lead)/(x_lead_r - x_lead)*(j - x_lead);
%     end
%     dzdt(i_global)   = a.*(z_foh( (i-1)*Nh/Na+1:(i)*Nh/Na )-z(i_global));
%     dzdt(x_lead)     = k*(0-z_lead);

    for j = i_global
        if j == 400
            hellooo = 1;
        end
        if j == 1
            dzdt(j) = v*(z(j+1) - z(j))/h  + a*z(j) - k*(z_foh(j)) + kr*r(j);
        elseif j == Nh
            dzdt(j) = v*(-z(j-1) + z(j))/h + a*z(j) - k*(z(j)) + kr*r(j) ;
        elseif j == x_lead
            dzdt(j) = v*(z(j+1) - 2*z(j) + z(j-1))/h^2 + a*z_lead - k*(z_lead) + kr*r(j);
        else
            dzdt(j) = v*(z(j+1) - 2*z(j) + z(j-1))/h^2 + a*z(j) - k*(z_foh(j)) + kr*r(j);
        end
    end
end

end

















