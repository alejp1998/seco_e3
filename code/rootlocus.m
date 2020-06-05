clear; clc;
warning('off')
s = tf('s');

% TRANSFER FUNCTION FROM EXPERIMENTAL MODEL
Km = 1361.48; % CONSTANT
pm = 28.692; % POLE
Gm = Km/(s*(s+pm));  % Open loop motor TF


% CONTROLLERS

% COEFICIENTS
%Integral
tau_i_pi = 0.2;
%Derivative
tau_d_pd = 1/100;
%Both
tau_i_pid = 0.2; % Integral coeff
tau_d_pid = 1/100; % Derivative coeff

%TRANSFER FUNCTIONS
Gc_p = 1; 
Gc_pd = 1 + tau_d_pd*s;
Gc_pi = 1 + 1/(tau_i_pi*s); 
Gc_pid = 1 + tau_d_pid*s + 1/(tau_i_pid*s);

%We select the type of controller we want
Gc = Gc_pid;

%OPEN LOOP SYSTEM TRANSFER FUNCTION
sys = Gc*Gm;

% ROOT LOCUS WHERE Kp VARIES
% Parameters
zeta = 0.7; % Damping constant
wn = 100; % Distance to origin (higher distance to real axis means more speed)
Kp_range = [1:0.01:10];

%SUBPLOT 1
f1 = subplot(2,1,1);
% Plot
rlocus(sys);
sgrid(zeta,wn);
hold on

selection_type = input('Select by points(0) or Kp values(1):');
if selection_type == 0 
    % Loop a user-specified number of times to find poles and corresponding
    % characteristics from root-locus plot
    nK = input('Enter number of CL-poles on which you wish to click to find corresponding gain, K(up to 5):  ');
    n  = 1;
    Kps = zeros(1, 20);
    while(n <= nK)
        [Kp, poles] = rlocfind(sys);
        
        Kps(n) = Kp;
        zeta_selected = abs(cos(angle(poles(1))));
        wn_selected = abs(poles(1));
        wd_selected = abs(poles(1)*cos(angle(poles(1))));

        fprintf('--- POINT %i CHARACTERISICS ---\n',n);
        fprintf('Kp = %3.3f \n', Kp);
        fprintf('Damping Constant = %.3f \n', zeta_selected);
        fprintf('wn = %3.3f \n', wn_selected);
        fprintf('wd = %3.3f \n', wd_selected);
        fprintf('-------------------------------\n\n');

        n = n + 1;
    end
else 
    % Introduce desired number of Kp points
    nK = input('Enter number of Kp values to analyze(up to 5): ');
    n  = 1;
    Kps = zeros(1, 20);
    while(n <= nK)
        fprintf('Value of Kp %i: ',n);
        Kp = input('');
        Kps(n) = Kp;
        [poles, Kp] = rlocus(sys,Kps(n));
        
        plot(poles,'x','Markersize',20);
        hold on
        zeta_selected = abs(cos(angle(poles(1))));
        wn_selected = abs(poles(1));
        wd_selected = abs(poles(1)*cos(angle(poles(1))));
        
        fprintf('--- VALUE %i CHARACTERISICS ---\n',n);
        fprintf('Kp = %3.3f \n', Kp);
        fprintf('Damping Constant = %.3f \n', zeta_selected);
        fprintf('wn = %3.3f \n', wn_selected);
        fprintf('wd = %3.3f \n', wd_selected);
        fprintf('-------------------------------\n\n');
        
        n = n + 1;
    end
    
end 
hold off
legend('RLocus',['Kp=' num2str(Kps(1))],['Kp=' num2str(Kps(2))],['Kp=' num2str(Kps(3))],['Kp=' num2str(Kps(4))],['Kp=' num2str(Kps(5))]);

%SUBPLOT 2
f2 = subplot(2,1,2);

%Select one of the shown points
index = input('Point to show (0 to show all) :  ');

if index == 0
    for i = 1:nK
        %With selected Kp, we create the continuous time transfer function
        sys_cl = feedback(Kps(i)*sys,1);
        den = tf((Kps(i)*sys)/(1+Kps(i)*sys)).den{:};
        fprintf('--- POINT %i ROUTH STABILITY ANALYSIS ---\n',i);
        routh_stability(den)
        fprintf('-----------------------------------------\n\n');
        %And we plot that response
        step(sys_cl);
        hold on
    end
    hold off
    legend(['Kp=' num2str(Kps(1))],['Kp=' num2str(Kps(2))],['Kp=' num2str(Kps(3))],['Kp=' num2str(Kps(4))],['Kp=' num2str(Kps(5))]);
else
    Kp = Kps(index);
    %With selected Kp, we create the continuous time transfer function
    sys_cl = feedback(Kps(i)*sys,1);
    fprintf('--- POINT %i ROUTH STABILITY ANALYSIS ---\n',index);
    routh_stability(den)
    fprintf('-----------------------------------------\n\n');
    %And we plot that response
    step(sys_cl);
end

function routh_stability(coeffVector)
% ROUTH STABILITY ANALYSIS
%The Routh-Hurwitz stability criterion is a necessary (and frequently
%sufficient) method to establish the stability of a single-input,
%single-output (SISO), linear time invariant (LTI) control system. 
%More generally, given a polynomial, some calculations using only the 
%coefficients of that polynomial can lead us to the conclusion that it
%is not stable.
%in this program you must give your system coefficents and the
%Routh-Hurwitz table would be shown
%
%   Farzad Sagharchi ,Iran
%   2007/11/12

% Taking coefficients vector and organizing the first two rows
ceoffLength = length(coeffVector);
rhTableColumn = round(ceoffLength/2);
%  Initialize Routh-Hurwitz table with empty zero array
rhTable = zeros(ceoffLength,rhTableColumn);
%  Compute first row of the table
rhTable(1,:) = coeffVector(1,1:2:ceoffLength);
%  Check if length of coefficients vector is even or odd
if (rem(ceoffLength,2) ~= 0)
    % if odd, second row of table will be
    rhTable(2,1:rhTableColumn - 1) = coeffVector(1,2:2:ceoffLength);
else
    % if even, second row of table will be
    rhTable(2,:) = coeffVector(1,2:2:ceoffLength);
end

% Calculate Routh-Hurwitz table's rows
%  Set epss as a small value
epss = 0.01;
%  Calculate other elements of the table
for i = 3:ceoffLength
   
    %  special case: row of all zeros
    if rhTable(i-1,:) == 0
        order = (ceoffLength - i);
        cnt1 = 0;
        cnt2 = 1;
        for j = 1:rhTableColumn - 1
            rhTable(i-1,j) = (order - cnt1) * rhTable(i-2,cnt2);
            cnt2 = cnt2 + 1;
            cnt1 = cnt1 + 2;
        end
    end
    
    for j = 1:rhTableColumn - 1
        %  first element of upper row
        firstElemUpperRow = rhTable(i-1,1);
        
        %  compute each element of the table
        rhTable(i,j) = ((rhTable(i-1,1) * rhTable(i-2,j+1)) - ....
            (rhTable(i-2,1) * rhTable(i-1,j+1))) / firstElemUpperRow;
    end
    
    
    %  special case: zero in the first column
    if rhTable(i,1) == 0
        rhTable(i,1) = epss;
    end
end

%  Compute number of right hand side poles(unstable poles)
%   Initialize unstable poles with zero
unstablePoles = 0;
%   Check change in signs
for i = 1:ceoffLength - 1
    if sign(rhTable(i,1)) * sign(rhTable(i+1,1)) == -1
        unstablePoles = unstablePoles + 1;
    end
end
%   Print calculated data on screen
fprintf('\n Routh-Hurwitz Table:\n')
rhTable
%   Print the stability result on screen
if unstablePoles == 0
    fprintf('~~~~~> it is a stable system! <~~~~~\n')
else
    fprintf('~~~~~> it is an unstable system! <~~~~~\n')
end
fprintf('\n Number of right hand side poles =%2.0f\n',unstablePoles)

sysRoots = roots(coeffVector);
fprintf('\n Given polynomial coefficients roots :\n')
sysRoots

end