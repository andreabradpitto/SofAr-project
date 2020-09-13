function J = km(T0e, T_vec, qtype)
 
    n = length(qtype);
    
    J = zeros(6,n);
    
    OOe = T0e(1:3,4);
    
    for i = 1 : n
        T0toi = eye(4,4); % T matrix from 0 to i
        for l = 1 : i
            % T0toi = prod of previous T matrices
            T0toi = T0toi * T_vec(:,:,l);
        end
        
        % ki projected on 0
        ki = T0toi(1:3,3);
        
        % distance from O to Oi
        OOi = T0toi(1:3,4);
        
        if qtype(i) == 0 % rot joint
            J(1:3, i) = ki;
            rei = OOe - OOi; % ray vector from i to e
            J(4:6, i) = cross(ki, rei);
            
        else % prismatic joint
            J(1:3, i) = zeros(3,1);
            J(4:6, i) = ki;
        end
    end
end