function [T_vec, T0e] = gm2 (T_vec)
    
    n_plus_1 = size(T_vec,3);
    
    T0e = eye(4,4);
    for i = 1 : n_plus_1
        T0e = T0e * T_vec(:,:,i);
    end
    
end
