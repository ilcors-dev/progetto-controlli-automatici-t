# Controllo del motore di un’automobile 🚗

Progetto di Controlli Automatici T - Ingegneria Informatica - Alma Mater Studiorum (Università di Bologna)

$$
\begin{array}
{| c | c |}\hline \text{ Parametri progetto}\\\hline
\gamma_1 & 0.75 \\\hline
\gamma_2 & 0.15 \\\hline
\beta & 1.3 \\\hline
\psi & 0.04 \\\hline
\delta_1 & 3*10^4 \\\hline
\delta_2 & 0.2 \\\hline
\delta_3 & 0.02 \\\hline
J & 20 \\\hline
\omega_e & 30
\\\hline
\end{array}
$$

## Presentazione del progetto

### Dinamica del sistema

$$
\dot{m} = γ_1(1 − \cos(βθ − ψ)) − γ_2ωm \\ \ \\ J\dot{ω} = δ_1m − δ_2ω − δ_3ω^2
$$

dove:

- $\theta(t)$ indica l’angolo di accelerazione
- $γ_1(1 − \cos(βθ − ψ))$ modella la caratteristica intrinseca della valvola
- $J$ rappresenta il momento d’inerzia equivalente del sistema automobile
- $\delta_1m$ descrive la coppia trasmessa all’albero motore
- $\delta_2\omega$ modella l’attrito nel motore
- $\delta_3\omega^2$ descrive la resistenza dell’aria

con $\gamma_1\in\mathbb{R},\psi\in\mathbb{R},J\in\mathbb{R},\delta_1\in\mathbb{R},\delta_2\in\mathbb{R},\delta_3\in\mathbb{R}$

![Untitled](Controllo%20del%20motore%20di%20un%20automobile%20-%20Progetto%20c%20553174a7952440aabb6e8442e40f2907/Untitled.png)

## Sistema in forma di stato

Considerando la velocità angolare $\omega(t)$ come uscita del sistema, possiamo scrivere la forma di stato del sistema in questo modo:

$$
x_1 = m \ ,\  x_2 = \omega
$$

$$
\dot{x} =f(x,u)=\begin{bmatrix} \dot{x_1} \\ \dot{x_2} \end{bmatrix}=\begin{bmatrix} f_1(x,u) \\ f_2(x,u) \end{bmatrix}=\begin{bmatrix} γ_1(1 − \cos(βu − ψ)) − γ_2x_1x_2\\ \frac{1}{J} (δ_1x_1 − δ_2x_2 − δ_3x_2^2)\end{bmatrix}\in\mathbb{R^2}
$$

$$
y = h(x,u)= \begin{bmatrix}x_2  \end{bmatrix}\in\mathbb{R}
$$

### Ricerca dell’equilibrio

Utilizzando la rappresentazione in forma di stato del nostro sistema e i parametri forniti, ricerchiamo l’intera coppia di equilibrio $(x_e,u_e)$ del sistema:

Dato l’equilibrio $\omega_e = 30$, ricaviamo il valore di equilibrio per la massa $(x_1)$ dalla seguente formula:

$$
f_2(x_e,u_e)=0 \\\to \frac{1}{J} (δ_1m_e − δ_2\omega_e − δ_3\omega_e^2)=0 \\\to m_e = \frac{1}{\delta_1}(\delta_2\omega_e+δ_3\omega_e^2) = 8 \ \cdot 10^{-4}
$$

Grazie agli equilibri degli stati appena trovati, dalla prima componente del sistema, possiamo ricavare l’ingresso d’equilibrio:

$$
f_1(x_e,u_e) = 0 \\\to γ_1(1 − \cos(βu − ψ)) − γ_2m_e\omega_e = 0 \\\to u_e = \frac{1}{\beta}(\arccos(1 - \frac{γ_2}{γ_1}\cdot\omega_e\cdot m_e)+ψ) \approx 0.1062
$$

Coppia di equilibrio:

$$
x_e = \begin{bmatrix}m_e\\\omega_e\end{bmatrix}=\begin{bmatrix} 8 \ \cdot 10^{-4}\\ 30\end{bmatrix} \ , \ u_e = 0,1062
$$

### Linearizzazione nell’intorno dell’equilibrio

$$
\delta\dot{x}(t)=A\delta x(t)+B\delta u(t) \\ \delta y(t)=C\delta x(t)+D\delta u(t)
$$

$$
\delta\dot{x}(t)=
\begin{bmatrix}
\frac{\partial f_1(x,u)}{\partial x_1} & \frac{\partial f_1(x,u)}{\partial x_2}
\\\\
\frac{\partial f_2(x,u)}{\partial x_1} & \frac{\partial f_2(x,u)}{\partial x_2}
\end{bmatrix}

\begin{bmatrix}
\delta x_1
\\\\
\delta x_2 \end{bmatrix}
+
\begin{bmatrix}
\frac{\partial f_1(x,u)}{\partial u}
\\\\
\frac{\partial f_2(x,u)}{\partial u}
\end{bmatrix}

\begin{bmatrix}
\delta u
\end{bmatrix}

\\ \ \\

\delta y(t)=
\begin{bmatrix}
\frac{\partial h(x,u)}{\partial x_1}
\\\\
\frac{\partial h(x,u)}{\partial x_2}
\end{bmatrix}

\begin{bmatrix}
\delta x_1
\\\\
\delta x_2 \end{bmatrix}
+
\begin{bmatrix}
0
\end{bmatrix}

\begin{bmatrix}
\delta u
\end{bmatrix}
$$

$$
A=
\left.\begin{bmatrix}
\frac{\partial f_1(x,u)}{\partial x_1} & \frac{\partial f_1(x,u)}{\partial x_2}
\\\\
\frac{\partial f_2(x,u)}{\partial x_1} & \frac{\partial f_2(x,u)}{\partial x_2}
\end{bmatrix}\right|_{\substack{x_1=m_e\\\\x_2=\omega_e}}
=
\begin{bmatrix}
-4,5 & -1,2*10^{-4}
\\\\
1500 & -0,07
\end{bmatrix}
\ , \
B=
\left.\begin{bmatrix}
\frac{\partial f_1(x,u)}{\partial u}
\\\\
\frac{\partial f_2(x,u)}{\partial u}
\end{bmatrix}\right|_{\substack{u=u_e}}
=
\begin{bmatrix}
0.0954
\\\\
0
\end{bmatrix}
\\ \ \\
C=
\left.\begin{bmatrix}
\frac{\partial h(x,u)}{\partial x_1}
&
\frac{\partial h(x,u)}{\partial x_2}
\end{bmatrix}\right|_{\substack{x_1=m_e\\\\x_2=\omega_e}}
=
\begin{bmatrix}
0 & 1
\end{bmatrix}
\ , \
D=
\begin{bmatrix}
0
\end{bmatrix}
$$

## Funzione di trasferimento

Dopo aver trovato il sistema linearizzato nell’intorno di equilibrio, si ricava la funzione di trasferimento grazie alla seguente formula:

$$
G(s) = C(sI-A)^{-1}B +D
$$

Possiamo ricavare la matrice inversa come segue:

$$
(sI-A)^{-1}=\frac{adj(sI-A)}{det(sI-A)}\to G(s)=C\ \frac{adj(sI-A)}{det(sI -A)}B+D
$$

$$
(sI -A)=\begin{bmatrix}
s+4,5 & +1,2 \cdot 10^{-4}
\\\\
-1500 & s+0,07
\end{bmatrix}
$$

Sfruttando le potenzialità offerte da Matlab, ricaviamo la matrice aggiunga (definita come la trasposta della matrice dei cofattori) e il determinante definiti come segue:

$$
cofattori(sI -A)=\begin{bmatrix}
s+0,07 & 1500
\\\\
+1,2\cdot 10^{-4}& s+4,5
\end{bmatrix}
$$

$$
adj(sI -A)=\begin{bmatrix}
s+0,07 & +1,2 \cdot 10^{-4}
\\\\
1500 & s+4,5
\end{bmatrix}
$$

$$
det(sI-A)= (s+4,5)\cdot(s+0,07)-(+1,2 \cdot 10^{-4} \cdot -1500) = s^2+4.57s+0.495
$$

partendo dai dati precedenti ricaviamo

$$
G(s)=
\frac{
C \cdot adj(sI-A) \cdot B
}{det(sI-A)}+C \\ \ \\

=
\frac{
\begin{bmatrix}
0 & 1
\end{bmatrix}\begin{bmatrix}
s+0,07 &+1,2\cdot 10^{-4}
\\\\
1500 & s+4,5
\end{bmatrix} \begin{bmatrix}
0.0954
\\\\
0
\end{bmatrix}
}{s^2+4.57s+0.495}+ \begin{bmatrix}
0
\end{bmatrix} \\ \ \\

=
\frac{
\begin{bmatrix}
1500
&
 s+4,5
\end{bmatrix}
}{s^2+4.57s+0.495} \cdot \begin{bmatrix}
0.0954
\\\\
0
\end{bmatrix}

\\ \ \\

=\begin{bmatrix}
\frac{1500}{s^2+4.57s+0.495} & \frac{ s+4,5}{s^2+4.57s+0.495}
\\\\
\end{bmatrix}
\begin{bmatrix}
0.0954
\\\\
0
\end{bmatrix}
\\ \ \\

= \frac{1500 \cdot 0.0954}{s^2+4.57s+0.495}
\\ \ \\

= \frac{123.1}{s^2+4.57s+0.495}
$$

Le radici del denominatore, ovvero i poli del sistema risultano essere :

$$
p1 = -4.458 \ ,\ p2=-0.111
$$

Riscriviamo i poli trovati come costanti di tempo

$$
T_1 = -\frac{1}{p_1} = 0.224 \ ,\ T_2=-\frac{1}{p_2} = 9.01
$$

Riscriviamo la $G(s)$ come

$$
G(s)=\frac{143.1}{s^2+4.57s+0.495} =\frac{143.1}{(s-4.458)(s-0.111)}
\\\ \\
$$

Di seguito è possibile visionare il codice Matlab per poter ricavare facilmente la funzione di trasferimento del sistema preso in esame.

```matlab
%% Definiamo le matrici nel punto di equilibrio trovato
%% [m_e,w_e] = [8e-4,30]
A = [-gamma_2*x_2e, -gamma_2*x_1e;
    delta_1/J     , -delta_2/J-(2*delta_3*x_2e)/J];
B = [beta*gamma_1*sin(beta*u_e-phi); 0];
C = [0, 1];
D = 0;

%% Funzione di trasferimento
s = tf('s');
[N, D] = ss2tf(A, B, C, D);
G = tf(N,D);
```

## Progetto del regolatore $R(s)=R_s(s)R_d(s)$

![Untitled](Controllo%20del%20motore%20di%20un%20automobile%20-%20Progetto%20c%20553174a7952440aabb6e8442e40f2907/Untitled%201.png)

1. Errore a regime $|e_\infty|\leq e^*=0.01$ in risposta a un gradino $\omega(t)=9\cdot1(t)$ e $d(t)=8\cdot1(t)$
2. Per garantire una certa robustezza del sistema si deve avere un margine di fase $M_f\geq55°$
3. Il sistema può accettare una sovraelongazione percentuale al massimo dell’$5\%:S\%\leq5\%$
4. Il tempo di assestamento all’$\epsilon\%=5\%$ deve essere inferiore al valore fissato: $T_{a,\epsilon}=0.08s$
5. l disturbo sull’uscita $d(t)$, con una banda limitata nel range di pulsazioni $[0,0.05]$, deve essere abbattuto di almeno $55db$
6. Il rumore di misura $n(t)$, con una banda limitata nel range di pulsazioni $[8\cdot10^3,2\cdot10^6]$, deve essere abbattuto di almeno $45db$

## Regolatore statico $R_s(s)=\frac{\mu_s}{s^k}$

### [Punto 3.1](https://www.notion.so/Controllo-del-motore-di-un-automobile-Progetto-c2-d48111ae8edf4346ad906a1d0028e2e8)

Partendo dalla funzione di trasferimento $G(s)$ ricavata al punto precedente, progettiamo il regolatore che rispetti le seguenti specifiche:

Scriviamo la funzione di trasferimento in anello aperto

$$
L(s) = R(s)G(s)
$$

dove possiamo scrivere $R(s) = R_s(s)R_d(s)$ con $R_s(s)= \frac{\mu_s}{s^k}$

$$
e_\infty = \frac{D+W}{1+\mu} \to \mu = \frac{D+W}{e_{\infty}} - 1
$$

la formula è valida solamente quando il numero di poli($k$) in $s=0$ della $L(s)$ è uguale a 0.

$$
R_s(s)=\frac{\mu_s}{s^0}=\mu_s
$$

$\mu = \mu_s\mu_g = L(0)=R(0)G(0)$ ,

possiamo quindi ricavare

$$
e_\infty = \frac{D+W}{1+\mu} \leq 0.01 \to \frac{17}{1+\mu}\leq 0.01 \\ \ \\
\mu \geq \frac{17}{0.01}-1 \geq 1699
$$

quindi

$$
\mu_s=\frac{L(0)}{G(0)}=\frac{\mu}{\mu_g}=\frac{1699}{289.1376}=5.8761
$$

Definiamo la nuova funzione di trasferimento estesa:

$$
G_e(s)=R_s(s)G(s)=\mu_sG(s)
$$

Di seguito è possibile visionare il codice Matlab.

```matlab
mu_s = ((DD + WW) / e_star) - 1;

% L(s)=R(s)G(s) -> mu=L(0)=R(0)G(0) -> R(0)=mu/G(0)
% guadagno minimo del regolatore ottenuto come L(0)/G(0)
G_0 = abs(evalfr(GG, 0));
RR_s = mu_s / G_0; % RR_s = 5.88

% Sistema esteso
GG_e = RR_s * GG;
```

## Regolatore dinamico

### [Punto 3.2 - Punto 3.3](https://www.notion.so/Controllo-del-motore-di-un-automobile-Progetto-c2-d48111ae8edf4346ad906a1d0028e2e8)

Dal grafico della seguente equazione

$$
S\%=100e^{-\frac{\pi\xi}{\sqrt{1-\xi^2}}}
$$

![Untitled](Controllo%20del%20motore%20di%20un%20automobile%20-%20Progetto%20c%20553174a7952440aabb6e8442e40f2907/Untitled%202.png)

![Untitled](Controllo%20del%20motore%20di%20un%20automobile%20-%20Progetto%20c%20553174a7952440aabb6e8442e40f2907/Untitled%203.png)

Ricaviamo lo smorzamento $\xi^*$affinché venga rispettato il requisito sulla sovraelongazione percentuale di $S\% \le 5\%$.

Da ciò che si evince dal grafico, il valore dello smorzamento minimo desiderato è $\xi^* \cong 0.69$ . Data la seguente equazione $M_f \ge 100 \xi^*$, possiamo concludere che il **margine di fase** desiderato è:

$$
M_f \ge 100\xi^* \to M_f \ge 69\%
$$

Notiamo che il valore di $M_f$ trovato **è più elevato rispetto a quello richiesto dalle specifiche**, ciò garantirà performance del sistema migliori.

### [Punto 3.4](https://www.notion.so/Controllo-del-motore-di-un-automobile-Progetto-c2-d48111ae8edf4346ad906a1d0028e2e8)

Il **tempo di assestamento** al 5% deve essere minore di $T^*=0.08s$, possiamo quindi trovare $w_{c,min}$ tramite la seguente formula:

$$
T_{a,5}<T^*
$$

dove

$$
T_{a,5}\cong\frac{3}{\xi\omega_n}\cong\frac{3}{\xi\omega_c}
$$

di conseguenza

$$
\frac{3}{\xi\omega_{c,min}}<T^*
\to
\frac{3}{\xi\omega_{c,min}}<0.08
\to
\omega_{c,min}>\frac{3}{\frac{M_f}{100}\cdot0.08}
\\ \to
\omega_{c,min}>\frac{300}{69.1\cdot0.08}
\\ \to \omega_{c,min}>54.27
$$

Per i punti [3.5](https://www.notion.so/Controllo-del-motore-di-un-automobile-Progetto-c2-d48111ae8edf4346ad906a1d0028e2e8) e [3.6](https://www.notion.so/Controllo-del-motore-di-un-automobile-Progetto-c2-d48111ae8edf4346ad906a1d0028e2e8) andremo a sfruttare il principio di sovrapposizione degli effetti, che indica la seguente:

$$
Y(s)=Y_\omega(s)+Y_d(s)+Y_n(s)
$$

### [Punto 3.5](https://www.notion.so/Controllo-del-motore-di-un-automobile-Progetto-c2-d48111ae8edf4346ad906a1d0028e2e8)

Specifica per il disturbo in uscita $d(t)$.

> Per il principio di sovrapposizione degli effetti ,‘ignoriamo’ l’uscita con ingresso $W(t)$ e $N(s)$ ponendo $W(s)=N(S)=0$

Sappiamo che un generico disturbo in uscita si presenta in questa forma:

$$
d(t)=D\cdot cos(\omega_d t+\psi_d)
$$

la funzione di sensitività risulta essere:

$$
S(s)=\frac{1}{1-R(s)G(s)}=\frac{1}{1-L(s)}
$$

quindi la relativa uscita del disturbo sarà

$$
y_d(t)=|S(j\omega_d)|\cdot Dcos(\omega_d t+\psi_d+arg(S(j\omega_d)))
$$

Per rispettare la specifica dovremo imporre $|S(j\omega_d)|_{db}\leq-55db$.
Sapendo che $|S(j\omega)|_{db}\approx -|L(j\omega)|_{db}$
a **basse frequenze**, possiamo ricavare il vincolo sull’ampiezza della $L(s)$ nel range di frequenze $[0, 0.05]$ come

$$
-|L(j\omega)|_{db}\leq-55db
\to
|L(j\omega)|_{db}\ge55db
$$

### [Punto 3.6](https://www.notion.so/Controllo-del-motore-di-un-automobile-Progetto-c2-d48111ae8edf4346ad906a1d0028e2e8)

Specifica per il rumore di misura $n(t)$.

> I passaggi effettuati sono analoghi a quelli per la $d(t)$

> Per il principio di sovrapposizione degli effetti ,‘ignoriamo’ l’uscita con ingresso $W(t)$ e $D(s)$ ponendo $W(s)=D(S)=0$

Viene richiesta un’attenuazione del rumore di misura di $45db$. Sapendo che la funzione di sensitività corrispondente è

$$
F(s)=\frac{R(s)G(s)}{1-R(s)G(s)}=\frac{L(s)}{1-L(s)}
$$

Dovremo quindi imporre $|F(j\omega_n)|_{db}\leq-45db$.

Sapendo che $|F(j\omega_n)|_{db}\approx|L(J\omega_n)|_{db}$ ad **alte frequenze**, possiamo ricavare il vincolo sull’ampiezza della $L(s)$ nel range di frequenze $[8\cdot 10^3, 2\cdot 10^6]$ come

$$
|L(J\omega_n)|_{db}\le-45db
$$

Possiamo ora tracciare il diagramma di Bode della $G_e(s)$ con le patch dei vincoli trovati

![Untitled](Controllo%20del%20motore%20di%20un%20automobile%20-%20Progetto%20c%20553174a7952440aabb6e8442e40f2907/Untitled%204.png)

---

Da una prima analisi del diagramma di Bode prodotto dal nostro sistema, emerge la vicinanza allo scenario B visto durante le lezioni: nell’intervallo di pulsazioni ammissibili per la pulsazione di attraversamento $\omega_c$, non esistono pulsazioni in cui la fase di $G_e(j\omega)$ rispetta il vincolo sul margine di fase.

Progettiamo quindi il regolatore dinamico per il nostro sistema di controllo utilizzando la formula che definisce la rete anticipatrice:

$$
R_d(s) = \frac{1+\tau s}{1 + \alpha \tau s}\  con \ 0 \lt \alpha \lt 1
$$

consideriamo la funzione della rete anticipatrice in $s=j\omega_c^*$ otteniamo

$$
R_d(j\omega_c^*) = M^*e^{j\varphi^*} \to
\\ \  \\
\frac{1+j\tau\omega_c^*}{1+j\alpha\tau\omega_c^*} = M^*(cos \varphi^*+jsin\varphi^*)
$$

valgono le seguenti uguaglianze

$$
\tau=\frac{M^*-cos\varphi^*}{\omega_c^*sin\varphi^*}
\ \ ,\ \
\alpha\tau=\frac{cos\varphi^*-\frac{1}{M^*}}{\omega_c^* sin\varphi^*}
$$

Quindi il nostro obiettivo è trovare i valori di $M^*$ e $\varphi^*$ in modo da ricavare $\tau$ e $\alpha \tau$ ,
imponiamo $|G_e(j\omega_c^*)|_{dB} + 20\ log \ M^*= 0$ , $M_f^* = 180° + arg(G_e(jω_c^*)) + \varphi^*$

e considerando i valori

$$
\omega_c^*=200 \ ,\ M_f^*=69
\\ \ \\ arg(G_e(j\omega_c^*)) = -178,69
\ ,\
|G_e(j\omega_c^*)|_{dB}=-33,54
$$

così facendo otteniamo i valori

$$
M^* = 10^{- \frac{|G_e(j\omega_c^*)|_{dB}}{20}} \approx 47,535
\\ \  \\
\varphi^* = \frac{\pi}{180}(M_f^* - 180° - arg(G_e(jω_c^*))) = 1,1814
$$

in fine i valori di $\tau$ e $\alpha \tau$ sono

$$
\tau \approx 0.2549\ rad \ ,\ \alpha \tau \approx 0.0019 \ rad
$$

codice MATLAB relativo

```matlab
% Rete anticipatrice

Mf_star = Mf_spec; % Mf_star = 69
omega_c_star = 200;
[mag_omega_c_star, arg_omega_c_star, omega_c_star] = bode(GG_e, omega_c_star)

mag_omega_c_star_dB = 20 * log10(mag_omega_c_star)

M_star = 10^(-mag_omega_c_star_dB / 20)
phi_star = Mf_star - 180 - arg_omega_c_star;

% Formule di inversione
tau = (M_star - cos(phi_star * pi / 180)) / omega_c_star / sin(phi_star * pi / 180)
alpha_tau = (cos(phi_star * pi / 180) - inv(M_star)) / omega_c_star / sin(phi_star * pi / 180)
alpha = alpha_tau / tau

R_d = (1 + tau*s)/(1 + alpha*tau*s); % rete anticipatrice
```

```matlab
% Test parametri

if M_star <= 1
    disp('Errore: M_start non soddisfa le specifiche (M_star > 1)')
    return;
end

phi_star_rad = phi_star*pi/180
if phi_star_rad < 0 | phi_star_rad > pi/2
    disp('Errore: phi_star non soddisfa le specifiche: 0<phi_star<pi/2')
    return;
end

check_flag = cos(phi_star*pi/180) - inv(M_star)
if check_flag < 0
    disp('Errore: alpha negativo');
    return;
end
```

![Untitled](Controllo%20del%20motore%20di%20un%20automobile%20-%20Progetto%20c%20553174a7952440aabb6e8442e40f2907/Untitled%205.png)

Il regolatore è fisicamente realizzabile in quanto il numero di poli del regolatore è uguale al numero di zeri e da ciò consegue che la pendenza dell’ampiezza di $L(s)$ è uguale alla pendenza della $G(s)$.

```matlab
% check regolatore fisicamente realizzabile
% poli-zero >= 0
check_reg = R_d * RR_s;

if size(pole(check_reg)) - size(zero(check_reg)) < 0
    fprintf('Il regolatore NON è fisicamente realizzabile!');
    return;
else
    fprintf('Il regolatore è fisicamente realizzabile!');
end
```

## Testing del sistema di controllo

Dopo aver opportunamente realizzato il regolatore per il nostro sistema linearizzato, andiamo a testare il nostro sistema con i seguenti segnali:

- $w(t) = 0.75 \cdot 1(t)$
- $d(t) = \sum_{k=1}^{4} 0.05 \cdot \sin(0.01kt)$
- $n(t) = \sum_{k=1}^{4}0.02 \cdot \sin(8 \cdot 10^{3}kt)$

![Untitled](Controllo%20del%20motore%20di%20un%20automobile%20-%20Progetto%20c%20553174a7952440aabb6e8442e40f2907/Untitled%206.png)

Nel precedente diagramma è possibile visionare l’uscita del nostro sistema sollecitato dall’ingresso $\omega(t) = 0.75 \cdot 1(t)$ e con le componenti $d(t)$ e n(t) nulle. Le patch verdi delimitano l’area in cui la risposta al nostro gradino dovrà assestarsi affinchè vengano rispettati i requisiti.

![Untitled](Controllo%20del%20motore%20di%20un%20automobile%20-%20Progetto%20c%20553174a7952440aabb6e8442e40f2907/Untitled%207.png)

Nel precedente diagramma è possibile visionare l’uscita del nostro sistema sollecitato dall’ingresso $d(t) = \sum_{k=1}^{4} 0.05 \cdot \sin(0.01kt)$ e con le componenti $w(t)$ e n(t) nulle.

![Untitled](Controllo%20del%20motore%20di%20un%20automobile%20-%20Progetto%20c%20553174a7952440aabb6e8442e40f2907/Untitled%208.png)

![Untitled](Controllo%20del%20motore%20di%20un%20automobile%20-%20Progetto%20c%20553174a7952440aabb6e8442e40f2907/Untitled%209.png)

Nel precedente diagramma rappresentiamo il comportamento del rumore di misura ad una sollecitazione in ingresso pari a $n(t) = \sum_{k=1}^{4}0.02 \cdot \sin(8 \cdot 10^{3}kt)$ quando gli ingressi $\omega(t)$ e $d(t)$ sono nulli. Come possiamo vedere, il rumore viene parzialmente attenuato.

![Untitled](Controllo%20del%20motore%20di%20un%20automobile%20-%20Progetto%20c%20553174a7952440aabb6e8442e40f2907/Untitled%2010.png)

Nel precedente diagramma, sfruttando la proprietà di sovrapposizione degli effetti, rappresentiamo l’uscita del nostro sistema come combinazione lineare delle uscite precedenti: $y(t) = y_\omega(t) + y_n(t) + y_d(t)$.

## Testing del sistema di controllo modello non lineare

![Untitled](Controllo%20del%20motore%20di%20un%20automobile%20-%20Progetto%20c%20553174a7952440aabb6e8442e40f2907/Untitled%2011.png)

Dopo aver realizzato il sistema non lineare su SimuLink e collegati i rispettivi segnali, effettuiamo il testing del sistema:

![Untitled](Controllo%20del%20motore%20di%20un%20automobile%20-%20Progetto%20c%20553174a7952440aabb6e8442e40f2907/Untitled%2012.png)

Il diagramma precedente illustra la risposta del sistema non lineare, a partire dall’intorno di equilibrio, ad un gradino di ampiezza data dalle specifiche del progetto ($9\cdot1(t)$).

Dopo una prima esecuzione, emerge come il comportamento del sistema non linearizzato diverga notevolmente.

Cambiando l’ampiezza del segnale in ingresso, raggiungiamo il comportamento desiderato: il nuovo setting prevede un ampiezza $10^{-4}\cdot1(t)$ per il segnale sinusoidale in ingresso.

A questo ingresso il sistema risponde correttamente assestandosi nell’intorno dell’equilibrio.

![Untitled](Controllo%20del%20motore%20di%20un%20automobile%20-%20Progetto%20c%20553174a7952440aabb6e8442e40f2907/Untitled%2013.png)
