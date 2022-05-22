%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controllo del motore di un’automobile                                                %
% Bernardini Claudio                                                                   %
% Corsetti Luca                                                                        %
% Straccali Leonardo                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARAMETRI DEL PROGETTO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
gamma_1 = 0.75;
gamma_2 = 0.15;
beta = 1.3;
phi = 0.04;
delta_1 = 3 * 10^4;
delta_2 = 0.2;
delta_3 = 0.02;
J = 20;
omega_e = 30;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARAMETRI DEL PROGETTO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Dinamica del sistema
% m_dot = gamma_1*(1-cos(beta*theta-phi))-gamma_2*omega*m
% J*omega = delta_1*m-delta_2*omega-delta_3*omega^2

%% LEGENDA DATI
% theta -> angolo di accelerazione
% gamma_1*(1-cos(beta*theta-phi)) -> caratteristica intrinseca della valvola.
% J -> momento d'inerzia
% delta_1*m -> coppia trasmessa all'albero motore
% delta_2*omega -> modella l'attrito nel motore
% delta_3*omega^2 -> descrive la resistenza dell'aria

%% PUNTO 1 - Sistema in forma di stato

x_2e = omega_e;
x_1e = (delta_2 * x_2e + delta_3 * x_2e^2) / delta_1;

u_e = (acos(- (gamma_2 / gamma_1) * 30 * 8e-4 + 1) + phi) / beta;

f_1 = gamma_1 * (1 - cos(beta * u_e - phi)) - gamma_2 * x_1e * x_2e;
f_2 = 1 / J * (delta_1 * x_1e - delta_2 * x_2e - delta_3 * x_2e^2);

%% Sistema linearizzato
% x_dot = A*x + B*u
% y = C*x + D*u

% Definiamo le matrici nel punto di equilibrio trovato [m_e,w_e]=[8e-4,30]
A = [-gamma_2 * x_2e, -gamma_2 * x_1e;
    delta_1 / J, -delta_2 / J - (2 * delta_3 * x_2e) / J];
B = [beta * gamma_1 * sin(beta * u_e - phi); 0];
C = [0, 1];
D = 0;

%% PUNTO 2 - Funzione di trasferimento
s = tf('s');
[NS, DS] = ss2tf(A, B, C, D);
GG = tf(NS, DS);

% oppure facendo i conti..
% sIA = [s+4.5, -1.2*10e-4;
%       -1500 ,  s+0.07];

% cof = [s+0.07  , 1500;
%       1.2*10e-4, s+4.5];

% adj = transpose(cof);

% check_N = (C*adj*B);
% check_D = ((s+4.5)*(s+0.07)) - (+1.2*1e-4*-1500);

% G_check = check_N / check_D
GG_poles = pole(GG);

% mostra la funzione di trasferimento e i poli
if 0
    GG
    GG_poles
    %    G_check;
    %    pole(G_check);
    return;
end

%% Diagramma di Bode di G(s)

% Definizione dell'intervallo di frequenze del diagramma di Bode
omega_plot_min = 10e-2;
omega_plot_max = 10e5;

figure(1);
bode(GG, {omega_plot_min, omega_plot_max});
grid on, zoom on;

% return;

%% PUNTO 3 - Progetto di un regolatore
% SPECIFICHE
% 1) Errore a regime |e∞|≤e* = 0.01 in risposta a un gradino w(t)=9·1(t) e d(t)=8·1(t)
% 2) Per garantire una certa robustezza del sistema si deve avere un margine di fase Mf ≥ 55°.
% 3) Il sistema può accettare una sovraelongazione percentuale al massimo dell%5% : S% ≤ 5%.
% 4) Il tempo di assestamento all'ε% = 5% deve essere inferiore al valore fissato: Ta,ε = 0.08s.
% 5) Il disturbo sull'uscita d(t), con una banda limitata nel range di pulsazioni [0,0.05], deve essere abbattutto di almeno 55 dB.
% 6) Il rumore di misura n(t), con una banda limitata nel range di pulsazioni [8 · 10^3, 2 · 10^6], deve essere abbattutto di almeno 45 dB.

% PUNTO 1
% ampiezze gradini
WW = 9;
DD = 8;

% errore a regime
e_star = 0.01;

% PUNTO 2
Mf_spec = 55;

% PUNTO 3 & 4
% Sovraelongazione massima e tempo di assestamento all'5%
s_100_spec = 0.05;
T_a1_spec = 0.08;

% PUNTO 5
% attenuazione disturbo sull'uscita
A_d = 55;
omega_d_MAX = 0.05;

% PUNTO 6
% attenuazione disturbo di misura
A_n = 45;
omega_n_MIN = 1e3;
omega_n_MAX = 1e6;

%% Regolatore statico

% valore minimo prescritto per L(0)
% sappiamo che e_inf = (D + W) / (1 + mu)
% mu = mu_s*mu_g dove mu_g è fisso ed è dato da G(s)
% pertanto mu = (D + W) / e_star - 1
% mu = (DD + WW) / e_star - 1;

% per trovaare mu_g è necessario riscrivere G(s) come
% G(s) = mu_g / ((1+T_1s)*(1+T_2s)) dove
% T_1s e T_2s sono i poli trovati sotto forma di costanti di tempo
% T_1s = -1 / GG_poles(1);
% T_2s = -1 / GG_poles(2);

% eguagliando le G(s) trovate possiamo ricavare mu_g
% G(s) = 123.1 / (s^2+4.57s+0.495) = mu_g/((s-4.458)(s-0.111))
% mu_g = 123.1*((s-4.458)(s-0.111))/(s^2+4.57s+0.495)
% mu_g = 123.1 * (s-4.458)*(s-0.111)/(s^2+4.57s+0.495);

mu_s = (DD + WW) / (e_star) - 1;

% L(s)=R(s)G(s) -> mu=L(0)=R(0)G(0) -> R(0)=mu/G(0)
% guadagno minimo del regolatore ottenuto come L(0)/G(0)
G_0 = abs(evalfr(GG, 0));
RR_s = mu_s / G_0; % RR_s = 5.88

% Sistema esteso
GG_e = RR_s * GG;

if 0
    figure(2)
    h_GGe = bodeplot(GG_e);
    grid on, zoom on;

    return
end

figure(2)
% SPECIFICHE SU d
omega_d_min = 0.0001;
omega_d_MAX = 1;
Bnd_d_x = [omega_d_min; omega_d_MAX; omega_d_MAX; omega_d_min];
Bnd_d_y = [A_d; A_d; -150; -150];
patch(Bnd_d_x, Bnd_d_y, 'r', 'FaceAlpha', 0.2, 'EdgeAlpha', 0);
hold on;

if 0
    h_GGe = bodeplot(GG_e);
    grid on, zoom on;
    return
end

% SPECIFICHE SU n
omega_n_min = 1e3;
omega_n_MAX = 1e4;
Bnd_n_x = [omega_n_min; omega_n_MAX; omega_n_MAX; omega_n_min];
Bnd_n_y = [-A_n; -A_n; 100; 100];
patch(Bnd_n_x, Bnd_n_y, 'r', 'FaceAlpha', 0.2, 'EdgeAlpha', 0);
hold on;

if 0
    h_GGe = bodeplot(GG_e)
    grid on, zoom on;
    return
end

% SPECIFICHE SU u
% omega_u_min = 200;
% omega_u_MAX = 1e4;
% Bnd_u_x = [omega_u_min; omega_u_MAX; omega_u_MAX; omega_u_min];
% Bnd_u_y = [0; 0; 100; 100];
% patch(Bnd_u_x, Bnd_u_y, 'r', 'FaceAlpha', 0.2, 'EdgeAlpha', 0);
% hold on;

if 0
    h_GGe = bodeplot(GG_e)
    grid on, zoom on;
    return
end

% SPECIFICHE SU S%
xi = 0.69;
S_100 = 100 * exp(-pi * xi / sqrt(1 - xi^2))
Mf_spec = xi * 100
% return;

% SPECIFICHE SU T_a
omega_Ta_low = 1e-4; % lower bound just for the plot
omega_Ta_MAX = 300 / (Mf_spec * T_a1_spec); % omega_c >= 300/(Mf*T^*) = 300/(69.1*0.08) ~ 54.27

Bnd_Ta_x = [omega_Ta_low; omega_Ta_MAX; omega_Ta_MAX; omega_Ta_low];
Bnd_Ta_y = [0; 0; -150; -150];
patch(Bnd_Ta_x, Bnd_Ta_y, 'r', 'FaceAlpha', 0.2, 'EdgeAlpha', 0);
hold on;

h_GGe = bodeplot(GG_e)
grid on, zoom on;

% STOP qui per disegnare solo le specifiche sul GUADAGNO
if 0
    return;
end

omega_c_min = 20;
omega_c_MAX = 200;

phi_spec = Mf_spec - 180;
phi_low = -270; % lower bound just for the plot

Bnd_Mf_x = [omega_c_min; omega_c_MAX; omega_c_MAX; omega_c_min];
% Bnd_Mf_y = [-130; -130; -270; -270];
Bnd_Mf_y = [phi_spec; phi_spec; phi_low; phi_low];
patch(Bnd_Mf_x, Bnd_Mf_y, 'r', 'FaceAlpha', 0.2, 'EdgeAlpha', 0);
hold on;

% STOP qui per le specifiche
if 0
    return
end
