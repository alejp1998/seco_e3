function [Kp,tau_i,tau_d1,tau_d2,tau_d] = set_parametros(pm, Km, beta, beta2, zeta)
    
    Kp=(pm^2*(2*beta+1/(zeta^2)))/(beta2^2*Km);
    
    tau_i=(beta2*zeta^2*(2*beta+1/(zeta^2)))/(beta*pm);
    
    tau_d1=(beta2*(beta-beta2+2))/(pm*(2*beta+1/(zeta^2)));
    tau_d2 = pm/(Kp*Km);
    tau_d = tau_d1 + tau_d2;
end