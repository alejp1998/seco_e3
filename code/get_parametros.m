function [Mp, tp, tr, ts] = get_parametros(x,t,v)
    L=length(x);
    
    Mp=0;
    tr=t(L);
    tp=0;
    ts=t(L);
    
    for index=1:L
        if x(index)>Mp
            Mp=x(index);
            tp=t(index);
        end
        if x(index)>1+v
            ts=t(index);
        elseif x(index)<1-v
            ts=t(index);
        end
    end
    for index=1:L
        if x(index)>1
            tr=t(index);
            break
        end
    end

end