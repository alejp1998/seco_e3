%%% SCRIPT DE TRANSFORMACION DE PARAMETROS DE DISEÑO %%%
%%%%%%%%%%% A PARAMETROS DEL TELELABORATORIO %%%%%%%%%%%

clear; clc; clf;

% TRANSFER FUNCTION OF THE MOTOR TO ANALYZE
Km = 2652.28/23;       % CONSTANT
pm = 64.986;        % POLE

% parametros de diseño
beta2 = 10;
beta = 10;
zeta =  1/sqrt(2);
%zeta =  0.9;

% calculo de los parámetros del controlador
[Kp,tau_i,tau_d1,tau_d2,tau_d] = set_parametros(pm, Km, beta, beta2, zeta);


% Calculo de parametros del telelabo
T = 10/1000; %ms
Kd1=Kp*tau_d1/T;
Ki=Kp*T/tau_i;
Kd2=Kp*tau_d2/T;

%print para copy-paste
disp(num2str(Kp,10))
disp(num2str(Ki,10))
disp(num2str(Kd1,10))
disp(num2str(Kd2,10))


%% Rediseño real

%datos extraidos
I_real=0.3000;
I_simulada=0.0340;
v_simulada=30.4133;
v_real = 8.8903;
ts_simulado=0.2040;
ts_real=0.5300;
Mp_real=0.0304;
Mp_simulado=0.1255;

% recalculo de parametros
Kp_p=Kp*Mp_simulado/Mp_real;
Ki_p = Ki*I_simulada/I_real;
Kd1_p = Kd1*v_simulada/v_real;

% renovar tau_d2
tau_d2_p = pm/(Kp_p*Km);
Kd2_p=Kp*tau_d2_p/T;

%print para copy-paste
disp(num2str(Kp_p,10))
disp(num2str(Ki_p,10))
disp(num2str(Kd1_p,10))
disp(num2str(Kd2_p,10))

%% CHECK PARAmS

taud1_p = Kd1_p*T/Kp_p;
taui_p = Kp_p*T/Ki_p;
taud2_p = pm/(Kp_p*Km);
Kd2_p = Kp*tau_d2_p/T;
