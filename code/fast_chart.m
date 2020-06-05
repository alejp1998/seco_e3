%%%%%%%%%% SCRIPT FOR CHARTS WITH %%%%%%%%%%
%%%%%%%% REAL AND SIMULATED SIGNALS %%%%%%%%

clf; clc; clear;
leg = [];
%% SIMULADO
file = "myDat1.txt";

Tr = readtable(file);
ar = table2array(Tr);

hold on
L = length(ar(:,1));

xlabel("tiempo (s)")
ylabel("posicion (rad)")
xlim([0 1])
ar(:,2)=ar(:,2)*pi;
%plot(ar(:,1),ar(:,2))

[Mp, tp, tr, ts] = get_parametros(ar(:,2)/pi,ar(:,1),0.02);
leg = [leg "ts="+num2str(ts,4)];
%% REAL
file = "final2-MOTOR3POS";

Tr = readtable(file);
ar = table2array(Tr);

hold on
L = length(ar(:,1));

xlabel("tiempo (s)")
ylabel("posicion (rad)")
xlim([0 1])
%plot(ar(:,1),ones(L)*pi,'k--')
plot(ar(:,1),ar(:,2))

[Mp, tp, tr, ts] = get_parametros(ar(:,2)/pi,ar(:,1),0.02);
leg = [leg "ts="+num2str(ts,4)];
legend(leg,'location','southeast')
%% Plot integral
t = ar(:,1);
x = ar(:,2);

x = x(1:find(t==tr)-1);


L = length(x);
y = ones(L,1)*pi;
z = zeros(2*L,1);
tz = zeros(2*L,1);

Integral = 0;

for i=1:L
   Integral = Integral + (y(i)-x(i))*(t(i+1)-t(i));
   z(2*i-1)=x(i);
   z(2*i)=y(i);
   tz(2*i-1)=t(i);
   tz(2*i)=t(i);
end

t = t(1:find(t==tr)-1);
plot(tz,z,'lineWidth',2);
%disp(Integral)

%I_real=0.3000
%I_simulada=0.0340

%% Plot velocidad tr

x=ar(:,2);
t=ar(:,1);
v = zeros(L-1,1);
for i=1:L-1
   v(i)=(x(i+1)-x(i))/(t(i+1)-t(i));
end

velocidad = v(find(t==tr));

angulo = x(find(t==tr));
v = x(find(t==tr)+1)-x(find(t==tr));

long = 20;
y = zeros(long,1);
ti = zeros(long,1);
for i=1:long+1
    y(i)= angulo + v*(i-long/2);
    ti(i)= t(find(t==tr)+i-long/2);
end

plot(ti,y,'lineWidth',2);

%v_simulada=30.4133;
%v_real = 8.8903;

%% TS data

%ts_simulado=0.2040;
%ts_real=0.5300;
%Mp_simulado=0.0304
%Mp_real=0.1255


