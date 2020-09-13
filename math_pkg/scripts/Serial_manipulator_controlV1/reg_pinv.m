function [XSharp,y] = reg_pinv(X,thr,A,Q)
    
    persistent x;
    if isempty(x), x = 0; end
    % Read ETA from base workspace.
    % If no value is found there, set ETA = default value = 5.
    % (note: third parameter of evalin is undocumented feature)
    ETA = evalin('base','params.eta','5');
    
    id = eye(size(Q,1));
   	to_pinv = X'*A*X + ETA*(id-Q)'*(id-Q);
    [~,Sigma,V] = svd(to_pinv);
    Vr = size(V,1);
    l = rank(Sigma);
    
    P=zeros(Vr,Vr);
    
    for ii = 1 : l
        singval = Sigma(ii,ii);
        if singval < thr
            P(ii,ii) = 1 - singval / thr; % simple straight line reg function
            x = x+1;
        end
    end
    
    %A=eye(Xr);
    XSharp=pinv(to_pinv + V'*P*V ) * X'*A*A;
    y = x;

end