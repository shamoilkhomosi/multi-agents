function dzdt = maspde1(t, z, phiL, phiR, h, Nh, mu, a)

dzdt = zeros(Nh, 1);
% define the boundary conditions
z(1) = phiL*(1-exp(-5*t));
z(Nh+1) = phiR*(1-exp(-5*t));
dzdx   = zeros(Nh, 1);
d2zdx2 = zeros(Nh, 1);

for i=2:Nh
    % using first order centered finite difference estimate for each step in space
    % note: boundary conditions are not included in the loop
    dzdx(i)   = (z(i+1) - z(i-1))./(2*h);
    d2zdx2(i) = (z(i+1) - 2*z(i) + z(i-1))./h^2;
    
    % definition of PDE
    dzdt(i)   = mu .* d2zdx2(i) + a.*z(i);
end

end
