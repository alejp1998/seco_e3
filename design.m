clear; clc;
warning('off')
s = tf('s');

% TRANSFER FUNCTION OF THE MOTOR TO ANALYZE
Km = 2652.28; %2600; % CONSTANT
pm = 64.986; %65; % POLE
Gm = Km/(s*(s+pm));  % Open loop motor TF


%% P

% DESIGN PARAMETERS
%Damping constant
zetas = [0.3,0.707,0.9];
%Beta
betas = [0,0,0]; %Condición para que no sea integrativo
%Beta2
betas_2 = [2,2,2]; %Condición para que no sea derivativo

% COEFICIENTS
%Proportional
%Kp = 1;

k = 3; 
Gc = cell(k,k); 
Ge = cell(k,k);
for c = 1:k
    for r = 1:k
        %Vary one parameters with two fixed
        if c==1
            zeta = zetas(r);
            beta = betas(1);
            beta_2 = betas_2(1);
        elseif c==2
            zeta = zetas(1);
            beta = betas(r);
            beta_2 = betas_2(1);
        else
            zeta = zetas(1);
            beta = betas(1);
            beta_2 = betas_2(r);
        end
        
        %Depending on design parameters
        Kp = pm^2*(2*beta+1/zeta^2)/(beta_2^2*Km);
        tau_d = beta_2*(beta - beta_2 + 2)/(pm*(2*beta + 1/zeta^2)); %Nula por la condición beta_2 = 2 y beta = 0
        tau_i = beta_2*zeta^2*(2*beta+1/zeta^2)/(beta*pm); %Infinito por la condicion beta = 0

        %Add resulting fucntions to matrixes
        Gc{c,r} = Kp*Km/(s^2 + pm*s + Kp*Km);
        Ge{c,r} = s*(s + pm)/(s^2 + pm*s + Kp*Km); 
    end
end

%TRANSFER FUNCTION OF P CONTROLLER
Gc_p = Kp*Km/(s^2 + pm*s + Kp*Km);

%ERROR TRANSFER FUNCTION OF P CONTROLLER
Ge_p = s*(s + pm)/(s^2 + pm*s + Kp*Km); 

%SUBPLOTS VARIYING EVERY DESIGN PARAMETER WITH THE OTHERS FIXED
name = 'P';
gen_plots(name,Gc,Ge,zetas,betas,betas_2);


%% P-D

% DESIGN PARAMETERS
%Damping constant
zetas = [0.5,0.707,0.9];
%Beta
betas = [0,0,0]; %Condición para que no sea integrativo
%Beta2
betas_2 = [0.5,0.707,0.9]; %Debe ser distinto de 2 para que no sea un controlador P

% COEFICIENTS
%Proportional
%Kp = 1;
%Derivative
%tau_d = 0.2;

k = 3; 
Gc = cell(k,k); 
Ge = cell(k,k);
for c = 1:k
    for r = 1:k
        %Vary one parameters with two fixed
        if c==1
            zeta = zetas(r);
            beta = betas(1);
            beta_2 = betas_2(1);
        elseif c==2
            zeta = zetas(1);
            beta = betas(r);
            beta_2 = betas_2(1);
        else
            zeta = zetas(1);
            beta = betas(1);
            beta_2 = betas_2(r);
        end
        
        %Depending on design parameters
        Kp = pm^2*(2*beta+1/zeta^2)/(beta_2^2*Km);
        tau_d = beta_2*(beta - beta_2 + 2)/(pm*(2*beta + 1/zeta^2));
        tau_i = beta_2*zeta^2*(2*beta+1/zeta^2)/(beta*pm); %Infinito por la condicion beta = 0
        
        %Add resulting fucntions to matrixes
        Gc{c,r} = Km*Kp/(s^2 + (pm + Kp*Km*tau_d)*s + Kp*Km);
        Ge{c,r} = s*(s + pm + Kp*Km*tau_d)/(s^2 + (pm + Kp*Km*tau_d)*s + Kp*Km);
    end
end

%TRANSFER FUNCTION OF P-D CONTROLLER
Gc_pd = Km*Kp/(s^2 + (pm + Kp*Km*tau_d)*s + Kp*Km);

%ERROR TRANSFER FUNCTION OF P-D CONTROLLER
Ge_pd = s*(s + pm + Kp*Km*tau_d)/(s^2 + (pm + Kp*Km*tau_d)*s + Kp*Km);

%SUBPLOTS VARIYING EVERY DESIGN PARAMETER WITH THE OTHERS FIXED
name = 'P-D';
gen_plots(name,Gc,Ge,zetas,betas,betas_2);


%% PD

% DESIGN PARAMETERS
%Damping constant
zetas = [0.5,0.707,0.9];
%Beta
betas = [0,0,0]; %Condición para que no sea integrativo
%Beta2
betas_2 = [0.5,0.707,0.9]; %Debe ser distinto de 2 para que no sea un controlador P

% COEFICIENTS
%Proportional
%Kp = 1;
%Derivative
%tau_d = 0.2;

k = 3; 
Gc = cell(k,k); 
Ge = cell(k,k);
for c = 1:k
    for r = 1:k
        %Vary one parameters with two fixed
        if c==1
            zeta = zetas(r);
            beta = betas(1);
            beta_2 = betas_2(1);
        elseif c==2
            zeta = zetas(1);
            beta = betas(r);
            beta_2 = betas_2(1);
        else
            zeta = zetas(1);
            beta = betas(1);
            beta_2 = betas_2(r);
        end
        
        %Depending on design parameters
        Kp = pm^2*(2*beta+1/zeta^2)/(beta_2^2*Km);
        tau_d = beta_2*(beta - beta_2 + 2)/(pm*(2*beta + 1/zeta^2));
        tau_i = beta_2*zeta^2*(2*beta+1/zeta^2)/(beta*pm); %Infinito por la condicion beta = 0
        
        %Add resulting fucntions to matrixes
        Gc{c,r} = Km*Kp*(1 + tau_d*s)/(s^2 + (pm + Kp*Km*tau_d)*s + Kp*Km);
        Ge{c,r} = s*(s + pm)/(s^2 + (pm + Kp*Km*tau_d)*s + Kp*Km);
    end
end

%TRANSFER FUNCTION OF PD CONTROLLER
Gc_pd = Km*Kp*(1 + tau_d*s)/(s^2 + (pm + Kp*Km*tau_d)*s + Kp*Km);

%ERROR TRANSFER FUNCTION OF PD CONTROLLER
Ge_pd = s*(s + pm)/(s^2 + (pm + Kp*Km*tau_d)*s + Kp*Km);

%SUBPLOTS VARIYING EVERY DESIGN PARAMETER WITH THE OTHERS FIXED
name = 'PD';
gen_plots(name,Gc,Ge,zetas,betas,betas_2);


%% PI

% DESIGN PARAMETERS
%Damping constant
zetas = [0.5,0.707,0.9];
%Beta
betas = [0.5,1,1.5]; %Distinta de 0
%Beta2
betas_2 = [betas(1) + 2,betas(2) + 2,betas(3) + 2]; %Distinta de 2 e igual a (beta + 2)

% COEFICIENTS
%Proportional
%Kp = 1;
%Integrative
%tau_i = 0.2;

k = 3; 
Gc = cell(k,k); 
Ge = cell(k,k);
for c = 1:k
    for r = 1:k
        %Vary one parameters with two fixed
        if c==1
            zeta = zetas(r);
            beta = betas(1);
            beta_2 = beta + 2;
        elseif c==2
            zeta = zetas(1);
            beta = betas(r);
            beta_2 = beta + 2;
        else
            zeta = zetas(1);
            beta = betas(1);
            beta_2 = beta + 2;
        end
        
        %Depending on design parameters
        Kp = pm^2*(2*beta+1/zeta^2)/(beta_2^2*Km);
        tau_d = beta_2*(beta - beta_2 + 2)/(pm*(2*beta + 1/zeta^2)); %Nula por la condición beta_2 = beta + 2 para que sea PI
        tau_i = beta_2*zeta^2*(2*beta+1/zeta^2)/(beta*pm);
        
        %Add resulting fucntions to matrixes
        Gc{c,r} = Km*Kp*(s + 1/tau_i)/(s^2*(s + pm) + Km*Kp*(s + 1/tau_i));
        Ge{c,r} = s^2*(s + pm)/(s^2*(s + pm) + Km*Kp*(s + 1/tau_i));
    end
end

%TRANSFER FUNCTION OF PI CONTROLLER
Gc_pi = Km*Kp*(s + 1/tau_i)/(s^2*(s + pm) + Km*Kp*(s + 1/tau_i));

%ERROR TRANSFER FUNCTION OF PI CONTROLLER
Ge_pi = s^2*(s + pm)/(s^2*(s + pm) + Km*Kp*(s + 1/tau_i));

%SUBPLOTS VARIYING EVERY DESIGN PARAMETER WITH THE OTHERS FIXED
name = 'PI';
gen_plots(name,Gc,Ge,zetas,betas,betas_2);


%% PID

% DESIGN PARAMETERS
%Damping constant
zetas = [0.5,0.707,0.9];
%Beta
betas = [0.5,1,1.5]; %Distinta de 0 y de (beta_2 - 2)
%Beta2
betas_2 = [0.5,0.707,0.9]; %Distinta de 2 y de (beta + 2)

% COEFICIENTS
%Proportional
%Kp = 1;
%Integrative
%tau_i = 0.2;

k = 3; 
Gc = cell(k,k); 
Ge = cell(k,k);
for c = 1:k
    for r = 1:k
        %Vary one parameters with two fixed
        if c==1
            zeta = zetas(r);
            beta = betas(1);
            beta_2 = betas_2(1);
        elseif c==2
            zeta = zetas(1);
            beta = betas(r);
            beta_2 = betas_2(1);
        else
            zeta = zetas(1);
            beta = betas(1);
            beta_2 = betas_2(r);
        end
        
        %Depending on design parameters
        Kp = pm^2*(2*beta+1/zeta^2)/(beta_2^2*Km);
        tau_d = beta_2*(beta - beta_2 + 2)/(pm*(2*beta + 1/zeta^2));
        tau_i = beta_2*zeta^2*(2*beta+1/zeta^2)/(beta*pm);
        
        %Add resulting fucntions to matrixes
        Gc{c,r} = Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i))/(s^2*(s + pm) + Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i)));
        Ge{c,r} = s^2*(s + pm)/(s^2*(s + pm) + Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i)));
    end
end

%TRANSFER FUNCTION OF PI CONTROLLER
Gc_pid = Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i))/(s^2*(s + pm) + Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i)));

%ERROR TRANSFER FUNCTION OF PI CONTROLLER
Ge_pid = s^2*(s + pm)/(s^2*(s + pm) + Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i)));

%SUBPLOTS VARIYING EVERY DESIGN PARAMETER WITH THE OTHERS FIXED
name = 'PID';
gen_plots(name,Gc,Ge,zetas,betas,betas_2);


%% PI-D

% DESIGN PARAMETERS
%Damping constant
zetas = [0.5,0.707,0.9];
%Beta
betas = [0.5,1,1.5]; %Distinta de 0 y de (beta_2 - 2)
%Beta2
betas_2 = [0.5,0.707,0.9]; %Distinta de 2 y de (beta + 2)

% COEFICIENTS
%Proportional
%Kp = 1;
%Integral
%tau_i = 0.2;
%Derivative
%tau_d1 = 0.02;

k = 3; 
Gc = cell(k,k); 
Ge = cell(k,k);
for c = 1:k
    for r = 1:k
        %Vary one parameters with two fixed
        if c==1
            zeta = zetas(r);
            beta = betas(1);
            beta_2 = betas_2(1);
        elseif c==2
            zeta = zetas(1);
            beta = betas(r);
            beta_2 = betas_2(1);
        else
            zeta = zetas(1);
            beta = betas(1);
            beta_2 = betas_2(r);
        end
        
        %Depending on design parameters
        Kp = pm^2*(2*beta+1/zeta^2)/(beta_2^2*Km);
        tau_d = beta_2*(beta - beta_2 + 2)/(pm*(2*beta + 1/zeta^2));
        tau_i = beta_2*zeta^2*(2*beta+1/zeta^2)/(beta*pm);
        
        %Add resulting fucntions to matrixes
        Gc{c,r} = Km*Kp*(s + 1/tau_i)/(s^2*(s + pm) + Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i)));
        Ge{c,r} = s^2*(s + pm + Km*Kp*tau_d)/(s^2*(s + pm) + Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i)));
    end
end

%TRANSFER FUNCTION OF PI CONTROLLER
Gc_pid = Km*Kp*(s + 1/tau_i)/(s^2*(s + pm) + Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i)));

%ERROR TRANSFER FUNCTION OF PI CONTROLLER
Ge_pid = s^2*(s + pm + Km*Kp*tau_d)/(s^2*(s + pm) + Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i)));

%SUBPLOTS VARIYING EVERY DESIGN PARAMETER WITH THE OTHERS FIXED
name = 'PI-D';
gen_plots(name,Gc,Ge,zetas,betas,betas_2);


%% PID-D

% DESIGN PARAMETERS
%Damping constant
zetas = [0.5,0.707,0.9];
%Beta
betas = [0.5,1,1.5]; %Distinta de 0 y de (beta_2 - 2)
%Beta2
betas_2 = [0.5,0.707,0.9]; %Distinta de 2 y de (beta + 2)

% COEFICIENTS
%Proportional
%Kp = 1;
%Integral
%tau_i = 0.2;
%Derivative
%tau_d1 = 0.02;

k = 3; 
Gc = cell(k,k); 
Ge = cell(k,k);
for c = 1:k
    for r = 1:k
        %Vary one parameters with two fixed
        if c==1
            zeta = zetas(r);
            beta = betas(1);
            beta_2 = betas_2(1);
        elseif c==2
            zeta = zetas(1);
            beta = betas(r);
            beta_2 = betas_2(1);
        else
            zeta = zetas(1);
            beta = betas(1);
            beta_2 = betas_2(r);
        end
        
        Kp = pm^2*(2*beta + 1/zeta^2)/(beta_2^2*Km);
        tau_d1 = beta_2*(beta - beta_2 + 2)/(pm*(2*beta + 1/zeta^2));
        tau_i = beta_2*zeta^2*(2*beta + 1/zeta^2)/(beta*pm);
        %Dependant on the coefs above
        tau_d2 = -pm/(Kp*Km); %Condicion que hace el error frente a la parabola nulo
        tau_d = tau_d1 + tau_d2; %Definicion de tau_d conjunto como suma de ambos
        
        %Add resulting fucntions to matrixes
        Gc{c,r} = Km*Kp*tau_d1*(s^2 + s/tau_d1 + 1/(tau_d1*tau_i))/(s^2*(s + pm) + Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i)));
        Ge{c,r} = s^2*(s + pm + Km*Kp*tau_d2)/(s^2*(s + pm) + Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i)));
    end
end

%TRANSFER FUNCTION OF D|PID CONTROLLER
Gc_pidd = Km*Kp*tau_d1*(s^2 + s/tau_d1 + 1/(tau_d1*tau_i))/(s^2*(s + pm) + Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i)));

%ERROR TRANSFER FUNCTION OF D|PID CONTROLLER
Ge_pidd = s^2*(s + pm + Km*Kp*tau_d2)/(s^2*(s + pm) + Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i)));

%SUBPLOTS VARIYING EVERY DESIGN PARAMETER WITH THE OTHERS FIXED
name = 'PID-D';
gen_plots(name,Gc,Ge,zetas,betas,betas_2);


%% D|PID

% DESIGN PARAMETERS
%Damping constant
zetas = [0.5,0.707,0.9];
%Beta
betas = [0.5,1,1.5]; %Distinta de 0 y de (beta_2 - 2)
%Beta2
betas_2 = [0.5,0.707,0.9]; %Distinta de 2 y de (beta + 2)


% COEFICIENTS
%Proportional
%Kp = 1;
%Integral
%tau_i = 0.2;
%Derivative
%tau_d1 = 0.02;
%Depending on design parameters
k = 3; 
Gc = cell(k,k); 
Ge = cell(k,k);
for c = 1:k
    for r = 1:k
        %Vary one parameters with two fixed
        if c==1
            zeta = zetas(r);
            beta = betas(1);
            beta_2 = betas_2(1);
        elseif c==2
            zeta = zetas(1);
            beta = betas(r);
            beta_2 = betas_2(1);
        else
            zeta = zetas(1);
            beta = betas(1);
            beta_2 = betas_2(r);
        end
        
        Kp = pm^2*(2*beta + 1/zeta^2)/(beta_2^2*Km);
        tau_d1 = beta_2*(beta - beta_2 + 2)/(pm*(2*beta + 1/zeta^2));
        tau_i = beta_2*zeta^2*(2*beta + 1/zeta^2)/(beta*pm);
        %Dependant on the coefs above
        tau_d2 = pm/(Kp*Km); %Condicion que hace el error frente a la parabola nulo
        tau_d = tau_d1 + tau_d2; %Definicion de tau_d conjunto como suma de ambos
        
        %Add resulting fucntions to matrixes
        Gc{c,r} = Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i))/(s^2*(s + pm) + Km*Kp*tau_d1*(s^2 + s/tau_d1 + 1/(tau_d1*tau_i)));
        Ge{c,r} = s^2*(s + pm - Km*Kp*tau_d2)/(s^2*(s + pm) + Km*Kp*tau_d1*(s^2 + s/tau_d1 + 1/(tau_d1*tau_i)));
    end
end

%TRANSFER FUNCTION OF D|PID CONTROLLER
Gc_dpid = Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i))/(s^2*(s + pm) + Km*Kp*tau_d1*(s^2 + s/tau_d1 + 1/(tau_d1*tau_i)));

%ERROR TRANSFER FUNCTION OF D|PID CONTROLLER
Ge_dpid = s^2*(s + pm - Km*Kp*tau_d2)/(s^2*(s + pm) + Km*Kp*tau_d1*(s^2 + s/tau_d1 + 1/(tau_d1*tau_i)));

%SUBPLOTS VARIYING EVERY DESIGN PARAMETER WITH THE OTHERS FIXED
name = 'D|PID';
gen_plots(name,Gc,Ge,zetas,betas,betas_2);


%% FUNCTIONS

function gen_plots (name,Gc,Ge,zetas,betas,betas_2)
    %Plots the variation of the responses if we fix two design parameters and vary
    %the remaining one
    s = tf('s');
    
    %MONOMIC FUNCTIONS (we multiply them by s because step function multiplies
    %by 1/s automatically)
    step_func = s*1/s; 
    ramp_func = s*1/s^2;
    para_func = s*1/s^3;

    k = 3;
    
    for c = 1:k
        if c==1
            name2 = '\zeta';
            legnd1 = ['\zeta=' num2str(zetas(1)) ', \beta=' num2str(betas(1)) ', \beta_2=' num2str(betas_2(1))];
            legnd2 = ['\zeta=' num2str(zetas(2)) ', \beta=' num2str(betas(1)) ', \beta_2=' num2str(betas_2(1))]; 
            legnd3 = ['\zeta=' num2str(zetas(3)) ', \beta=' num2str(betas(1)) ', \beta_2=' num2str(betas_2(1))];
        elseif c==2
            name2 = '\beta';
            if name(1) == 'P' && name(2) == 'I' && length(name) == 2
                legnd1 = ['\zeta=' num2str(zetas(1)) ', \beta=' num2str(betas(1)) ', \beta_2=' num2str(betas_2(1))];
                legnd2 = ['\zeta=' num2str(zetas(1)) ', \beta=' num2str(betas(2)) ', \beta_2=' num2str(betas_2(2))]; 
                legnd3 = ['\zeta=' num2str(zetas(1)) ', \beta=' num2str(betas(3)) ', \beta_2=' num2str(betas_2(3))];
            else
                legnd1 = ['\zeta=' num2str(zetas(1)) ', \beta=' num2str(betas(1)) ', \beta_2=' num2str(betas_2(1))];
                legnd2 = ['\zeta=' num2str(zetas(1)) ', \beta=' num2str(betas(2)) ', \beta_2=' num2str(betas_2(1))]; 
                legnd3 = ['\zeta=' num2str(zetas(1)) ', \beta=' num2str(betas(3)) ', \beta_2=' num2str(betas_2(1))];
            end
        else
            name2 = '\beta_2';
            legnd1 = ['\zeta=' num2str(zetas(1)) ', \beta=' num2str(betas(1)) ', \beta_2=' num2str(betas_2(1))];
            legnd2 = ['\zeta=' num2str(zetas(1)) ', \beta=' num2str(betas(1)) ', \beta_2=' num2str(betas_2(2))]; 
            legnd3 = ['\zeta=' num2str(zetas(1)) ', \beta=' num2str(betas(1)) ', \beta_2=' num2str(betas_2(3))];
        end
        
        %STEP RESPONSE 
        subplot(4,3,c);
        
        %Find tmax
        tmax = 0;
        ymax = 0;
        ymin = 0;
        for r = 1:k
            [y,t] = step(Gc{c,r});
            if max(t) > tmax
                tmax = max(t);
            end
            if max(y) > ymax 
                ymax = max(y);
            end
            if min(y) < ymin
                ymin = min(y);
            end
        end
        t = (0:tmax/1000:tmax)';
        
        for r = 1:k
            [y,t] = step(Gc{c,r},t);
            plot(t, y, 'LineWidth', 2);
            hold on
        end
        
        hold off
        grid;
        axis([0 tmax ymin+ymin/20 ymax+ymax/20]);
        legend(legnd1,legnd2,legnd3);
        title(['STEP(' name2 ')']);

        %ERROR RESPONSE TO MONOTONIC FUNCTIONS
        %Step error
        subplot(4,3,c+3);
        
        %Find tmax
        tmax = 0;
        ymax = 0;
        ymin = 0;
        for r = 1:k
            [y,t] = step(step_func*Ge{c,r});
            if max(t) > tmax
                tmax = max(t);
            end
            if max(y) > ymax 
                ymax = max(y);
            end
            if min(y) < ymin
                ymin = min(y);
            end
        end
        t = (0:tmax/1000:tmax)';
        
        for r = 1:k
            [y,t] = step(step_func*Ge{c,r},t);
            plot(t, y, 'LineWidth', 2);
            hold on
        end
        hold off
        grid;
        legend(legnd1,legnd2,legnd3);
        axis([0 tmax ymin+ymin/20 ymax+ymax/20]);
        title(['STEP ERROR(' name2 ')']);

        %Ramp response
        subplot(4,3,c+6);
        
        %Find tmax
        tmax = 0;
        ymax = 0;
        ymin = 0;
        for r = 1:k
            [y,t] = step(ramp_func*Ge{c,r});
            if max(t) > tmax
                tmax = max(t);
            end
            if max(y) > ymax 
                ymax = max(y);
            end
            if min(y) < ymin
                ymin = min(y);
            end
        end
        t = (0:tmax/1000:tmax)';
        
        for r = 1:k
            [y,t] = step(ramp_func*Ge{c,r},t);
            plot(t, y, 'LineWidth', 2);
            hold on
        end
        hold off
        grid;
        legend(legnd1,legnd2,legnd3);
        axis([0 tmax ymin+ymin/20 ymax+ymax/20]);
        title(['RAMP ERROR(' name2 ')']);

        %Parabola response
        subplot(4,3,c+9);
        
        %Find tmax
        tmax = 0;
        ymax = 0;
        ymin = 0;
        for r = 1:k
            [y,t] = step(para_func*Ge{c,r});
            if max(t) > tmax
                tmax = max(t);
            end
            if max(y) > ymax 
                ymax = max(y);
            end
            if min(y) < ymin
                ymin = min(y);
            end
        end
        t = (0:tmax/1000:tmax)';
        
        for r = 1:k
            [y,t] = step(para_func*Ge{c,r},t);
            plot(t, y, 'LineWidth', 2);
            hold on
        end
        hold off
        grid;
        legend(legnd1,legnd2,legnd3);
        axis([0 tmax ymin+ymin/20 ymax+ymax/20]);
        title(['PARABOLA ERROR(' name2 ')']);
        
    end
    sgtitle([name,' CONTROLLER RESPONSES DEPENDING ON DESIGN PARAMETERS']);
    

end