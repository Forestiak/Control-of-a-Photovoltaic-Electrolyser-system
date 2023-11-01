function D = fcn(vpv, ipv)

    
    
    D_init = 0.4;
    D_max = 0.9;
    D_min = 0.1;
    deltaD = 20e-3;
    persistent Vold Pold Dold;
    
    dataType = "double";
    
    
    %Assign starting values 
    if isempty(Vold)
        Vold = 0;
        Pold = 0;
        Dold = D_init;
    end
    
    P = vpv*ipv;
    dV = vpv - Vold;
    dP = P - Pold;
    
    
    if dP ~= 0
        if dP < 0
            if dV < 0
                D = Dold - deltaD;
            else
                D = Dold + deltaD;
            end
            else
            if dV < 0
                D = Dold + deltaD;
            else
                D = Dold - deltaD;
            end
        end
    else D = Dold
    end
    
    if D > D_max
        D = D_max; 
    elseif D < D_min
        D = D_min;
    end
    
    Dold = D;
    Vold = vpv;
    Pold = P;
