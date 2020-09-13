function T_vec = gm1 (q, DH, qtype, Tne)

    % DH = each row has, in order, theta, d, a, alpha
    n = length(q);
    T_vec = zeros(4,4,n+1);
    
    for i = 1:n
        T_bar = transl(DH(i,3), 0, 0) * r2t(rotx(DH(i,4))) ...
           * r2t(rotz(DH(i,1))) * transl(0, 0, DH(i,2));
        
        % qtype(i) is 0 if rot, 1 if transl
        if (qtype(i) == 0)
            % rotational joint
            T_q = r2t(rotz(q(i)));
        elseif (qtype(i) == 1)
            % translational joint
            T_q = transl(0, 0, q(i));
        else
            error('joint not recognized');
        end
        
        % T_vec{1, i} is the transformation matrix from frame i - 1 to
        % frame i 
        T_vec(:,:,i) = T_bar * T_q;
    end
    
    T_vec(:,:,n+1) = Tne;
end
