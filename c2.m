%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controllo del motore di un automobile, Progetto C2                                   %
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
% J*omega_dot = delta_1*m-delta_2*omega-delta_3*omega^2

%% LEGENDA DATI
% theta -> angolo di accelerazione variabile d'ingresso
% gamma_1*(1-cos(beta*theta-phi)) -> caratteristica intrinseca della valvola.
% J -> momento d'inerzia
% delta_1*m -> coppia trasmessa all'albero motore
% delta_2*omega -> modella l'attrito nel motore
% delta_3*omega^2 -> descrive la resistenza dell'aria

%% PUNTO 1 - Sistema in forma di stato

x_2e = omega_e;
x_1e = (delta_2 * x_2e + delta_3 * x_2e^2) / delta_1;

u_e = (acos(- (gamma_2 / gamma_1) * x_2e * x_1e + 1) + phi) / beta;

f_1 = gamma_1 * (1 - cos(beta * u_e - phi)) - gamma_2 * x_1e * x_2e;
f_2 = 1 / J * (delta_1 * x_1e - delta_2 * x_2e - delta_3 * x_2e^2);

%% Sistema linearizzato
% x_dot = A*x + B*u
% y = C*x + D*u

% Definiamo le matrici nel punto di equilibrio trovato x_e=[m_e,w_e]=[8e-4,30]
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
% sI-A = [s+4.5, -1.2*10e-4;
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
omega_plot_min = 10e-6;
omega_plot_max = 10e6;

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
omega_d_min = 0.00001;
omega_d_MAX = 0.05;

% PUNTO 6
% attenuazione disturbo di misura
A_n = 45;
omega_n_MIN = 8 * 1e3;
omega_n_MAX = 2 * 1e6;

%% Regolatore statico

% valore minimo prescritto per L(0)
% sappiamo che e_inf = (D + W) / (1 + mu)
% mu = mu_s*mu_g dove mu_g è fisso ed è dato da G(s)
% pertanto mu = (D + W) / e_star - 1

% L(s)=R(s)G(s) -> mu=L(0)=R(0)G(0) -> mu_s=R(0)=mu/G(0)
% guadagno minimo del regolatore ottenuto come L(0)/G(0)
G_0 = abs(evalfr(GG, 0));
mu_s = ((DD + WW) / (e_star) - 1)/G_0;

RR_s = mu_s; % RR_s = 5.88

% Sistema esteso
GG_e = RR_s * GG; % GG_e = mu_s * GG

if 0
    figure(2)
    h_GGe = bodeplot(GG_e);
    grid on, zoom on;

    return
end

figure(2)
% PATCH sul diagramma di Bode

% SPECIFICHE SU d
Bnd_d_x = [omega_d_min; omega_d_MAX; omega_d_MAX; omega_d_min];
Bnd_d_y = [A_d; A_d; -200; -200];
patch(Bnd_d_x, Bnd_d_y, 'r', 'FaceAlpha', 0.2, 'EdgeAlpha', 0);
hold on;

if 0
    h_GGe = bodeplot(GG_e);
    grid on, zoom on;
    return
end

% SPECIFICHE SU n
Bnd_n_x = [omega_n_MIN; omega_n_MAX; omega_n_MAX; omega_n_MIN];
Bnd_n_y = [-A_n; -A_n; 100; 100];
patch(Bnd_n_x, Bnd_n_y, 'r', 'FaceAlpha', 0.2, 'EdgeAlpha', 0);
hold on;

if 0
    h_GGe = bodeplot(GG_e)
    grid on, zoom on;
    return
end

% SPECIFICHE SU S%
xi = 0.69;
S_100 = 100 * exp(-pi * xi / sqrt(1 - xi^2))
Mf_spec = xi * 100

% SPECIFICHE SU T_a
omega_Ta_low = 1e-8; % lower bound just for the plot
omega_Ta_MAX = 300 / (Mf_spec * T_a1_spec); % omega_c >= 300/(Mf*T^*) = 300/(69.1*0.08) ~ 54.27

Bnd_Ta_x = [omega_Ta_low; omega_Ta_MAX; omega_Ta_MAX; omega_Ta_low];
Bnd_Ta_y = [0; 0; -200; -200];
patch(Bnd_Ta_x, Bnd_Ta_y, 'r', 'FaceAlpha', 0.2, 'EdgeAlpha', 0);
hold on;

h_GGe = bodeplot(GG_e,{omega_plot_min, omega_plot_max})
grid on, zoom on;

% STOP qui per disegnare solo le specifiche sul GUADAGNO
if 0
    return;
end

omega_c_min = omega_Ta_MAX;
omega_c_MAX = omega_n_MIN;

phi_spec = Mf_spec - 180;
phi_low = -270; % lower bound just for the plot

Bnd_Mf_x = [omega_c_min; omega_c_MAX; omega_c_MAX; omega_c_min];
Bnd_Mf_y = [phi_spec; phi_spec; phi_low; phi_low];
patch(Bnd_Mf_x, Bnd_Mf_y, 'r', 'FaceAlpha', 0.2, 'EdgeAlpha', 0);
hold on;

% STOP qui per le specifiche
if 0
    return;
end

%% Design del regolatore dinamico

% Rete anticipatrice

Mf_star = Mf_spec; % Mf_star = 69
omega_c_star = 200;
[mag_omega_c_star, arg_omega_c_star, omega_c_star] = bode(GG_e, omega_c_star)

mag_omega_c_star_dB = 20 * log10(mag_omega_c_star)

M_star = 10^(-mag_omega_c_star_dB / 20) % M_star = 47.535
phi_star = Mf_star - 180 - arg_omega_c_star; % phi_star = 1.181
phi_star_rad = phi_star * pi / 180 % phi_star_rad = 206.193 rad

% Formule di inversione
tau = (M_star - cos(phi_star_rad)) / (omega_c_star * sin(phi_star_rad)) % tau = 0.2549 rad
alpha_tau = (cos(phi_star_rad) - inv(M_star)) / (omega_c_star * sin(phi_star_rad)) % alpha_tau = 0.0019 rad
alpha = alpha_tau / tau % alpha = 0.00745 rad

if M_star <= 1
    disp('Errore: M_start non soddisfa le specifiche (M_star > 1)')
    return;
end

if phi_star_rad < 0 | phi_star_rad > pi / 2
    disp('Errore: phi_star non soddisfa le specifiche: 0<phi_star<pi/2')
    return;
end

check_flag = cos(phi_star * pi / 180) - inv(M_star)

if check_flag < 0
    disp('Errore: alpha negativo');
    return;
end

% return;

%% Diagrammi di Bode con specifiche includendo regolatore dinamico

R_d = (1 + tau * s) / (1 + alpha * tau * s); % rete anticipatrice

LL = R_d * GG_e; % funzione di anello

% check regolatore fisicamente realizzabile
% poli-zero >= 0
check_reg = R_d * RR_s;

if size(pole(check_reg)) - size(zero(check_reg)) < 0
    fprintf('Il regolatore NON è fisicamente realizzabile!');
    return;
else
    fprintf('Il regolatore è fisicamente realizzabile!');
end

figure(3);
hold on;

% Legenda colori
Legend_mag = ["A_d"; "A_n"; "\omega_{c,min}"; "G(j\omega)"];

% Specifiche su ampiezza
patch(Bnd_d_x, Bnd_d_y, 'r', 'FaceAlpha', 0.2, 'EdgeAlpha', 0);
patch(Bnd_n_x, Bnd_n_y, 'g', 'FaceAlpha', 0.2, 'EdgeAlpha', 0);
patch(Bnd_Ta_x, Bnd_Ta_y, 'b', 'FaceAlpha', 0.2, 'EdgeAlpha', 0);
legend(Legend_mag);

% Plot Bode con margini di stabilità
margin(LL, {omega_plot_min, omega_plot_max});
grid on;

% Legenda colori
Legend_arg = ["G(j\omega)"; "M_f"];
legend(Legend_arg);

% Specifiche su fase
patch(Bnd_Mf_x, Bnd_Mf_y, 'g', 'FaceAlpha', 0.2, 'EdgeAlpha', 0);
hold on;
legend(Legend_arg);

% STOP qui per sistema con controllore dinamico + specifiche
if 0
    return;
end

%% Check prestazioni

% Funzione di sensitività
SS = 1 / (1 + LL);

% Funzione di sensitività complementare
FF = LL / (1 + LL);

% tt potrebbe essere suddiviso in più intervalli di cambionamento a seconda
% del caso di utilizzo, per comodità ne scegliamo uno unico
tt = (0:1e-4:1e3)'; 

%% Check prestazioni in anello chiuso
%% richiesta: w(t) = 0.75 * 1(t)

% Risposta al gradino
figure(4);

WW = 0.75;

T_simulation = 0.1;
[y_step, t_step] = step(WW * FF, T_simulation);
plot(t_step, y_step, 'b');
grid on, zoom on, hold on;

% vincolo sovraelongazione minore uguale all'5%
patch([0, T_simulation, T_simulation, 0], [WW * (1 + s_100_spec), WW * (1 + s_100_spec), WW + 1, WW + 1], 'r', 'FaceAlpha', 0.3, 'EdgeAlpha', 0.5);
ylim([0, WW + 1]);

% vincolo tempo di assestamento all'5%
LV = abs(evalfr(WW * FF, 0)); % valore limite gradino: W*F(0)
patch([T_a1_spec, T_simulation, T_simulation, T_a1_spec], [LV * (1 - 0.01), LV * (1 - 0.01), 0, 0], 'g', 'FaceAlpha', 0.1, 'EdgeAlpha', 0.5);
patch([T_a1_spec, T_simulation, T_simulation, T_a1_spec], [LV * (1 + 0.01), LV * (1 + 0.01), LV + 1, LV + 1], 'g', 'FaceAlpha', 0.1, 'EdgeAlpha', 0.1);

Legend_step = ["Risposta al gradino"; "Vincolo sovraelongazione"; "Vincolo tempo di assestamento"];
legend(Legend_step);

if 0
    return;
end

%% Check disturbo in uscita
figure(6);

% Simulazione disturbo a pulsazione 0.05
omega_d = 0.01;
syms k

DD = 0.05;
dd = 0;

% d(t) = sum_{k=1}^{4} 0.05 * sin(0.01kt)
for k = 1:4
    dd = dd + DD * sin(omega_d * tt * k);
end

y_d = lsim(SS, dd, tt);
hold on, grid on, zoom on
plot(tt, dd, 'm')
plot(tt, y_d, 'b')
grid on
legend('dd', 'y_d')

%% Check disturbo di misura

figure(7);

% Simulazione disturbo a pulsazione 8*10^3
omega_n = 8 * 1e3;
syms k

NN = 0.02;
nn = 0;

% n(t) = sum_{k=1}^{4} 0.02 * sin(8 * 10^3 * kt)
for k = 1:4
    nn = nn + NN * sin(omega_n * tt * k);
end

y_n = lsim(-FF, nn, tt);
hold on, grid on, zoom on
plot(tt, nn, 'm')
plot(tt, y_n, 'b')
grid on
legend('nn', 'y_n')

% Test uscita totale
y_step = step(WW * FF, tt);
figure(8);
y_tot = y_d + y_n + y_step;
plot(tt, y_tot)
