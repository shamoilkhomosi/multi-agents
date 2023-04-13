function dzdt = maspde_con(t, z, x, phiL, phiR, Na, h, Nh, k, a, v, kr, r)

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
%     dzdx(i)   = (z(i+1) - z(i-1))./(2*h);
%     d2zdx2(i) = (z(i+1) - 2*z(i) + z(i-1))./h^2;
    
%     dzdt(i_global)   = a.*(z_lead-z(i_global));
%     dzdt(x_lead)     = k*(0-z_lead);
    
    for j = i_global
        if j == 1
            dzdt(j) = v*(z(j+1) - z(j))/h  + a*z(j) - k*(z_lead) + kr*r(j);
        elseif j == Nh
            dzdt(j) = v*(-z(j-1) + z(j))/h + a*z(j) - k*(z(j)) + kr*r(j) ;
        elseif j == x_lead
            dzdt(j) = v*(z(j+1) - 2*z(j) + z(j-1))/h^2 + a*z_lead - k*(z_lead) + kr*r(j);
        else
            dzdt(j) = v*(z(j+1) - 2*z(j) + z(j-1))/h^2 + a*z(j) - k*(z_lead) + kr*r(j);
        end
    end
end

end