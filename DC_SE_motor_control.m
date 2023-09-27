%% DC SE Motor

close all
clear all
clc

%% System data

Vn=600;         % Line voltage [V]
omega_n=314;    % Motor rated speed [rad/s]
eta=0.9;        % Motor efficiency
tau_a=10e-3;    % Armature time constant [s]
Ve_n=120;       % Excitation circtuit rated voltage [V]
Ie_n=1;         % Excitation circtuit rated current [V]
tau_e=1;        % Excitation circtuit time constant [s]

% Parameters identification

M=10*1000+200*80;
Vmax=60*0.278;
a=(Vmax-0)/25;
Ftr=M*a;
Ptr=Ftr*Vmax;           % Traction power
Pfr=1/3*Ptr;            % Friction power
Pn=Ptr+Pfr;             % Rated power

In=Pn/(eta*Vn);
Tn=Pn/omega_n;

K=Tn/(In*Ie_n);         % Torque/emf constant

En=eta*Vn;              % Rated emf

Ra=(1-eta)*Vn/In;
La=Ra*tau_a;
Re=Ve_n/Ie_n;
Le=Re*tau_e;

J=M*Vmax^2/omega_n^2; 
B=Pfr/omega_n^2;

% Speed and torque conversion coefficient

C1=Vmax/omega_n;    % rad/s -> m/s
C2=omega_n/Vmax;    % m/s -> rad/s
C3=omega_n/60;      % km/h -> rad/s

Tfr=Pfr/omega_n;

%% Transfer function

s=tf('s');

% Armature
Ga=1/(Ra+s*La);
tauGa=La/Ra;
TaGa=5*tauGa;

% Excitation
Ge=1/(Re+s*Le);
tauGe=Le/Re;
TaGe=5*tauGe;

% Mechanical Load
Gm=1/(B+s*J);
tauGm=J/B;
TaGm=5*tauGm;

figure
bode(Ga)
title('Bode Diagrams')
grid on
hold on
bode(Ge)
bode(Gm)
grid off
legend('Ga','Ge','Gm')
hold off

%% Armature current controller

Tad=TaGa/2;
wia=5/Tad;

kp_curr=wia*La;
ki_curr=wia*Ra; 

%% Speed controller

ws=wia/200;

kp_speed=ws*J;
ki_speed=ws*B;

%% Excitation current controller

wie=wia/10;

kp_ecc=wie*Le;
ki_ecc=wie*Re;