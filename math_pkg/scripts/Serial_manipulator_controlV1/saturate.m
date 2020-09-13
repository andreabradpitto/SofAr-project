% Digital saturator

function q = saturate(q,qmin,qmax,alternative_min)
    for ii = 1:length(q)
        if q(ii) < qmin(ii)
            if exist('alternative_min')
                q(ii) = alternative_min;
            else
                q(ii) = qmin(ii);
            end
        elseif q(ii) > qmax(ii)
            q(ii) = qmax(ii);
        end
    end
end