%%%%%%%%%%% SCRTIP PARA EL ANALISIS DE %%%%%%%%%%%
%%%%%%%%%%% RAMPA PARABOLA Y SINUSOIDE %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% APARTADO 3 %%%%%%%%%%%%%%%%%%%

clf; clc; clf;

name = "parab2";
leg = [];
%% SIGNAL
file = name+"-MOTOR3POS";

Tr = readtable(file);
ar = table2array(Tr);

hold on
L = length(ar(:,1));

xlabel("tiempo (s)")
ylabel("posicion (rad)")
xlim([0 10])
%plot(ar(:,1),ones(L)*pi,'k--')
plot(ar(:,1),ar(:,2))

[Mp, tp, tr, ts] = get_parametros(ar(:,2)/pi,ar(:,1),0.02);
leg = [leg "posicion"];


%% ERROR

file = name +"-MOTOR3ERR";

Tr = readtable(file);
ar = table2array(Tr);

hold on
L = length(ar(:,1));

xlabel("tiempo (s)")
ylabel("posicion (rad)")
xlim([0 10])
plot(ar(:,1),ar(:,2))

leg = [leg "error"];


%% REF

file = name +"-MOTOR3REF";

Tr = readtable(file);
ar = table2array(Tr);

hold on
L = length(ar(:,1));

xlabel("tiempo (s)")
ylabel("posicion (rad)")
xlim([0 10])
plot(ar(:,1),ar(:,2))

leg = [leg "referencia"];


%% MAKE ERRORES CHART

L=12;
errores = zeros(L,1);
f = zeros(L,1);
for i=1:12
   file =  "f"+string(i)+"-MOTOR3ERR";
   
    Tr = readtable(file);
    ar = table2array(Tr);
    
    errores(i)=max(abs(ar(:,2)))
    f(i)=i/4;
end
plot(f, errores)
xlabel("frecuencia (Hz)")
ylabel("error maximo (rad)")

