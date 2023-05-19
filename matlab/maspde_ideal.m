function dzdt = maspde_ideal(t, z, x, phiL, phiR, Na, h, Nh, k, a, v, kr, r)

xsect = zeros(Na, Nh/Na);
for i=1:Na
    xsect(i,:) = x((i-1)*Nh/Na+1:(i)*Nh/Na);
end
dzdt = zeros(Nh, 1);

% persistent e
% if isempty(e)
%     e = zeros(Nh,1);
% else
%     e = e + (r-z);
% end
% dzdx   = zeros(Nh, 1);
% d2zdx2 = zeros(Nh, 1);

for i=1:Na
    i_global = (i-1)*Nh/Na+1:(i)*Nh/Na;
    x_lead = round(xsect(i,round(Nh/Na/2))/h)+1;
    z_lead = z(x_lead);
%     tic
    for j = i_global
        if j == 1 % robin BC - global + local
            dzdt(j) = v*(z(j+1) - z(j))/h + a*z(j) - k*(z(j)) + kr*r(j) - v*(r(j+1) - r(j))/h;
        elseif j == Nh
            dzdt(j) = v*(-z(j-1) + z(j))/h + a*z(j) - k*(z(j)) + kr*r(j) - v*(-r(j-1) + r(j))/h;
        elseif j == x_lead
            dzdt(j) = v*(z(j+1) - 2*z(j) + z(j-1))/h^2 + a*z_lead - k*(z_lead) + kr*r(j) - v*(r(j+1) - 2*r(j) + r(j-1))/h^2 ;
        else
            dzdt(j) = v*(z(j+1) - 2*z(j) + z(j-1))/h^2 + a*z(j) - k*(z(j)) + kr*r(j) - v*(r(j+1) - 2*r(j) + r(j-1))/h^2;
        end
    end
%     toc
end

end

















