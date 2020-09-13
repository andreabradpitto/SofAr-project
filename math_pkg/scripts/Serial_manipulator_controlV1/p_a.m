function Rho = p_a(ROg,TOe)
    %a
    % R = Rab
    %b
    
    ROe = TOe(1:3,1:3); % extract the rotation part
    Reg = ROe.' * ROg;
    
    % delta = costheta, rho = vtheta
    delta = (trace(Reg) - 1)/2; % from algebraic form of unit vector
 
    eie = [1; 0; 0];
    eje = [0; 1; 0];
    eke = [0; 0; 1];
    eig = Reg(1:3,1);
    ejg = Reg(1:3,2);
    ekg = Reg(1:3,3);
    
    % axb means a ^ b
    ixi = cross(eie, eig);
    jxj = cross(eje, ejg);
    kxk = cross(eke, ekg);
    sigma = ROe * 0.5 * (ixi + jxj + kxk); % from geometric form projected on frame O
    sigma_norm = norm(sigma);
    
    if (abs(delta - 1) < 1e-7) 
       v = [0; 0; 0];
       theta = 0;
    elseif (abs(delta + 1) < 1e-7)
        sumijk = eie + eje + eke + eig + ejg + ekg;
        v = sumijk / norm(sumijk);
        theta = pi;
    else
        v = sigma / sigma_norm;
        theta = atan2(sigma_norm, delta);
    end
    
    Rho = v*theta;
end
