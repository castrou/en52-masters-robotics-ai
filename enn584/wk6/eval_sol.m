function error = eval_sol(xr,map,mu)
    num_l = (length(mu)-3)/2;
    Z_sol = zeros(num_l,2);
    for i = 1:num_l        
        lidx = 2*i - 1;
        z = predicZ(mu(4:end),mu(1:3),lidx);
        Z_sol(i,:) = z';
    end
    Z_ref = zeros(num_l,2);
    map_v = map';
    map_v = map_v(:);
    xr_e  = xr(:,end);
    for i = 1:num_l        
        lidx = 2*i - 1;
        z = predicZ(map_v,xr_e,lidx);
        Z_ref(i,:) = z';
    end
    error = mean(abs(Z_sol(:,1) - Z_ref(:,1)));
end

function z = predicZ(map,xr,i)
        
        z = [0 0]';
        zx = map(i);
        zy = map(i+1);
        rx = xr(1);
        ry = xr(2);
        rth = xr(3);
        r = sqrt((rx-zx)^2 + (ry-zy)^2);
        b = atan2(zy-ry,zx-rx) - rth;        
        z(1) = r ;
        z(2) = b;
        z(2) = wrapToPi(z(2));
end