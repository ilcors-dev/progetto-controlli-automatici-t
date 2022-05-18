%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controllo del motore di un’automobile                                                %
% Bernardini Claudio                                                                   %
% Corsetti Luca                                                                        %
% Straccali Leonardo                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARAMETRI DEL PROGETTO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
gamma_1 = 0.75;
gamma_2 = 0.15;
beta = 1.3;
phi = 0.04;
delta_1 = 3*10^4;
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

x_2e = omega_e;
x_1e = (delta_2 * x_2e + delta_3*x_2e^2) / delta_1;

u_e = (acos(-(gamma_2/gamma_1)*30*8e-4+1)+phi)/beta;

f_1 = gamma_1*(1-cos(beta*u_e-phi))-gamma_2*x_1e*x_2e;
f_2 = 1/J*(delta_1*x_1e-delta_2*x_2e-delta_3*x_2e^2);

%% Sistema linearizzato
% x_dot = A*x + B*u
% y = C*x + D*u

% Definiamo le matrici nel punto di equilibrio trovato [m_e,w_e]=[8e-4,30]
A = [-gamma_2*x_2e, -gamma_2*x_1e;
    delta_1/J    , -delta_2/J-(2*delta_3*x_2e)/J];
B = [beta*gamma_1*sin(beta*u_e-phi); 0];
C = [0, 1];
D = 0;


%% Funzione di trasferimento
s = tf('s');
[N,D]=ss2tf(A, B, C, D);
G=tf(N,D);

if 0 
    G
    pole(G)
    return;
end

zpk(G);

% Definizione dell'intervallo di frequenze del diagramma di Bode
omega_plot_min=10^(-2);
omega_plot_max=10^5;

% Il diagramma di Bode presenta anche le limitazioni per il disturbo
% di misura (zona gialla)
figure();
patch([omega_e,omega_plot_max,omega_plot_max,omega_e],[-29.9,-29.9,100,100],'y','FaceAlpha',0.3,'EdgeAlpha',0);
hold on;
[Mag,phase,w]=bode(G,{omega_plot_min,omega_plot_max});
margin(Mag,phase,w);
grid on;
hold off;
title("Funzione di trasferimento iniziale");
% Come si nota G(s) attraversa la zona proibita 