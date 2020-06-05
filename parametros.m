%%%%% SCRIPT DE SINTONIZACION DE PARAMETROS %%%%%
%%%%%%%%%%%% Y GRAFICAS DEL APARTADO %%%%%%%%%%%%

clear; clc; clf;
warning('off')
s = tf('s');

% TRANSFER FUNCTION OF THE MOTOR TO ANALYZE
Km = 2652.28;       % CONSTANT
pm = 64.986;        % POLE
Gm = Km/(s*(s+pm)); % Open loop motor TF


%% Parametro hechos
v=0.02;
beta2 = 10;
beta = 10;
zeta =  1/sqrt(2);
[Kp,tau_i,tau_d1,tau_d2,tau_d] = set_parametros(pm, Km, beta, beta2, zeta);
Gc_dpid = Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i))/(s^2*(s + pm) + Km*Kp*tau_d1*(s^2 + s/tau_d1 + 1/(tau_d1*tau_i)));
[x,t] = step(Gc_dpid,1);
[Mp, tp, tr, ts] = get_parametros(x,t, v);

%% MAKE CSV

a = [t x];
T = array2table(a);
writetable(T,'myDat1.txt','Delimiter',' ');
type 'myDat1.txt'

%% PLOT
hold on
plot(t,x,'LineWidth',2);
plot(t,ones(length(t)),'k--');
xlabel("time (s)");
xlim([0 1])
%leg = "Mp="+num2str(Mp,3)+", tp="+num2str(tp,3)+", tr="+num2str(tr,3)+", ts="+num2str(ts,3);
%legend(leg,'location','southeast');
hold off

%% Grafica Mp 

v=0.02;

beta2 = 1;
Mps = [];
betas = 0:0.1:50;
eps=[0.5 1/sqrt(2) 1 2 5];


hold on
L=length(betas);

Mps = [];
leg = [];
for zeta=eps
    temp = [];
    for beta=betas
        [Kp,tau_i,tau_d1,tau_d2,tau_d] = set_parametros(pm, Km, beta, beta2, zeta);
        Gc_dpid = Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i))/(s^2*(s + pm) + Km*Kp*tau_d1*(s^2 + s/tau_d1 + 1/(tau_d1*tau_i)));
        [x,t] = step(Gc_dpid);
        [Mp, tp, tr, ts] = get_parametros(x,t, v);
        temp = [temp Mp];
    end
    leg = [leg, "zeta="+num2str(zeta, 3)];
    plot(betas,temp,'LineWidth',2);
    Mps = [Mps; temp];
end
legend(leg)

plot(betas,ones(L,1)*1.06,'k--','LineWidth',2);
plot(betas,ones(L,1)*1.13,'k--','LineWidth',2);
xlabel("beta")
ylabel("Mp")


%% Get valid beta

v=0.02;
beta2 = 1;
Mps = [];
betas = 0:0.01:50;
zeta = 1/sqrt(2);

for beta=betas
    [Kp,tau_i,tau_d1,tau_d2,tau_d] = set_parametros(pm, Km, beta, beta2, zeta);
    Gc_dpid = Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i))/(s^2*(s + pm) + Km*Kp*tau_d1*(s^2 + s/tau_d1 + 1/(tau_d1*tau_i)));
    [x,t] = step(Gc_dpid);
    [Mp, tp, tr, ts] = get_parametros(x,t, v);
    Mps = [Mps Mp];
end

L=length(betas);
valid_betas = [];
for i=1:L
    Mp = Mps(i);
    beta = betas(i);
    if Mp<1.06
        continue
    elseif Mp>1.13
        continue
    else
        valid_betas=[valid_betas beta];
    end
end

%% Check ts
v=0.02;
betas2 = 83.5:0.01:84.2;
beta = 10;
zeta = 1/sqrt(2);

tss = [];
trs = [];


for beta2=betas2
    [Kp,tau_i,tau_d1,tau_d2,tau_d] = set_parametros(pm, Km, beta, beta2, zeta);
    Gc_dpid = Km*Kp*tau_d*(s^2 + s/tau_d + 1/(tau_d*tau_i))/(s^2*(s + pm) + Km*Kp*tau_d1*(s^2 + s/tau_d1 + 1/(tau_d1*tau_i)));
    [x,t] = step(Gc_dpid);
    [Mp, tp, tr, ts] = get_parametros(x,t, v);
    tss = [tss ts];
    trs = [trs tr];
end

valid_betas2=betas2(trs<=0.25);
plot(valid_betas2)

%% Create graph
clf
hold on

len = length(betas2);
plot(betas2,tss,'b','LineWidth',2);
plot(betas2,trs,'r','LineWidth',2)

plot(betas2, ones(len)*0.4,'b--','LineWidth',1)
plot(betas2, ones(len)*0.25,'r--','LineWidth',1)

ylabel("tiempo (s)");
xlabel("beta_2");
legend(["ts" "tr"],'Location','northwest')

hold off