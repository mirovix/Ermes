%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                       %
%                       TEST 29-11-2021 ERMES                           %
%       TEST ELETTROVALVOLE PER TROVARE RELAZIONE SPINTA-PWM            %
%                                                                       %
% Obbiettivo: Ricavare una funzione di relazione tra spinta e tempo di  %
%             accensione delle elettrovalvole per il meccanismo di      %
%             controllo PWM                                             %
%                                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Spegazione teorica:
% La relazione tra spinta media e t_on nel controllo PWM è lineare,
% infatti è del tipo: 𝐹_𝑚𝑒𝑑𝑖𝑎 = 𝐹_𝑚𝑎𝑥 ∗ 𝑡_𝑜𝑛 / 𝑇_𝑠𝑎𝑚𝑝𝑙𝑒
% con t_on tempo di apertura compreso tra 0 e T_sample.
% La precisione del controllo PWM è definita dal minimo tempo di apertura
% delle elettrovalvole (per esempio se T_sample è di 0.1s e si hanno 8 bit
% per discretizzare l’intervallo, si ha che un bit rappresenta
% 0.1/8=12.5ms di accensione).
% Attenzione però, definendo un numero di bit per discretizzare
% l’intervallo di tempo T_sample, dobbiamo imporre che 
% T_sample > n_bit * t_on&off con t_on&off il tempo che
% ci mette una elettrovalvola ad aprirsi del tutto e chiudersi del tutto.
% Facciamo un esempio: assumiamo t_on&off = 8ms (4+4) avremo che
% 8ms*8=0.064ms che è circa 1/16Hz, per cui scegliamo 16Hz come la massima
% frequenza del nostro PWM, altrimenti con frequenze maggiori
% (e quindi T_pwm minori) potremmo avere intervalli in cui l’elettrovalvola
% non si apre e chiude completamente (portando a problemi di controllo,
% oscillazione della spinta, fluidodinamiche ecc). Inoltre, è importante
% sottolineare che piccoli intervalli del PWM portano ad una maggiore
% precisione di controllo perché si controlla in maniera più fina
% la spinta, potendo avere impulsi più piccoli (deltaV = S_max*t_on).
%
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    STUDIO DATI SPERIMENTALI
%
%     Procedura:
%
% 1)  Previa taratura
%
%     - Primo esperimento
% 2)  Settiamo una frequenza di PWM, testiamo più valori:
%     4,8,16 Hz (si raddoppia di volta in volta). 
%     -> Frequenze troppo basse portano ad acquisizioni dati troppo lunghe
%     che sprecano molta CO2. Se si aumenta la frequenza, la valvola non
%     fa in tempo ad aprirsi e chiudersi.
% 3)  Si inizia la presa dati, con una pressione settata di 2 bar (30 psi)
% 4)  L’Arduino accende l’elettrovalvola seguendo il controllo PWM, quindi
%     prima apre per un livello di precisione poi due, poi tre fino a 8.
%     Per ogni punto, la presa dati dura 0.5s (valore scelto), 
%     ossia in totale si ha una presa dati di 4s (0.5*8).
% 5)  L’Arduino da come output un vettore del tipo: 
%     [time; state; u] = [tempo da real time clock; stato delle valvole;
%     lettura della cella di carico]
% 6)  Un codice MATLAB media i dati, plotta e trova un grafico per ogni
%     frequenza di PWM
%
%     - Secondo esperimento
% 7)  Settiamo diverse frequenze di PWM (per esempio 8, 16, 24, 24)
% 8)  Con tutte le frequenze si apre per un tempo prefissato
% 9)  L’Arduino da come output un vettore del tipo: 
%     [frequenza PWM; lettura della cella di carico]
% 10) Plottiamo la spinta media al variare della frequenza di PWM%    
% 11) Cerchiamo a quale di queste la spinta media decresce in maniera
%     evidente e rappresenta la frequenza massima di PWM 
% 
%     - Terzo esperimento
% 12) Settiamo la frequenza molto alta
% 13) Cerchiamo di plottare la spinta quando la EV non si apre del tutto
% 14) L’Arduino da come output un vettore del tipo: 
%     [frequenza PWM settata; time; lettura della cella di carico]
% 15) Stimiamo un valore per il tempo di risposta
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%

close all
clear
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DALLA TARATURA

m122 = 98.590309688091990;
q122 = -8.245683452922070e+02;  %alla fine di tutto qua incolliamo il codice di taratura preciso
massa_equivalente = @(vv) q122 + m122*vv; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%          PRIMO ESPERIMENTO
%
%
%               DATI
%   - Frequenza di PWM variabile
%   - Pressione regolatore 30 psi
%   - Frequenza amplificatore 80 Hz
%   - 
%   - 
%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%
% %                           PRESA DATI


% 30 Hz
load('freq_experiment/high_frequency_results/r1_30Hz_0_256_16step.txt')
load('freq_experiment/high_frequency_results/r2_30Hz_0_256_16step.txt')
load('freq_experiment/high_frequency_results/r3_30Hz_0_256_16step.txt')
r30 = [r1_30Hz_0_256_16step; r1_30Hz_0_256_16step; r1_30Hz_0_256_16step];

% 122.55 Hz
load('freq_experiment/high_frequency_results/r1_122Hz_208_256_4step.txt')
load('freq_experiment/high_frequency_results/r2_122Hz_208_256_4step.txt')
load('freq_experiment/high_frequency_results/r3_122Hz_208_256_6step.txt')
r122 = [r1_122Hz_208_256_4step; r2_122Hz_208_256_4step; r3_122Hz_208_256_6step];

% 30 Hz
load('freq_experiment/high_frequency_results/r1_245Hz_208_256_4step.txt')
load('freq_experiment/high_frequency_results/r2_245Hz_208_256_4step.txt')
r245 = [r1_245Hz_208_256_4step; r2_245Hz_208_256_4step];

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       ANALISI DATI

%%%%%%%%%%%%%%%%%%%%%
% % 30 Hz
a=zeros(16,1); % indice 
for j=1:16
for i=1:length(r30)
    if  r30(i,1)==j-1
        a(j)=a(j)+1;
        k = a(j);
        r30_mat(k,j) = r30(i,2);
    end
end
end

for j=1:16
    r30_step = r30_mat(:,j);
    r30_step(r30_step==0)=[];   % toglie gli zeri dovuti alla presa dati
    r30_value(j) = mean(r30_step)/1e6;
end

% retta di regressione
xi30 = 4:1:16; % togliamo dove non sia apre
x_media30 = mean(xi30);
yi30 = r30_value(4:1:16);
y_media30 = mean(yi30);

scarti_x_vect30 = (xi30 - x_media30*ones(1,length(xi30)));
scarti2_x_vect30 = scarti_x_vect30.^2;
S_xx0 = 0;
for i = 1:length(scarti2_x_vect30)
Sxx = scarti2_x_vect30(i) + S_xx0;  % calcolo della varianza
S_xx0 = Sxx;
end

scarti_y_vect = (yi30 - y_media30*ones(1,length(yi30)));
S_xy0 = 0;
for i = 1:length(scarti2_x_vect30)
Sxy = scarti_x_vect30(i)*scarti_y_vect(i) + S_xy0; % calcolo della covarianza
S_xy0 = Sxy;
end

m30 = Sxy/Sxx;
q30 = y_media30 - m30*x_media30;
xx30 = linspace(4, 16, 100);
yy30 = q30 + m30*xx30;

% plotting
xp30 = 1:1:16;
yi = 9.81*massa_equivalente(r30_value);
yy30 = 9.81*massa_equivalente(yy30);

figure
plot(xp30,yi,'*')
hold on
plot(xx30, yy30)
plot(xp30,yi)
title('Grafico step PWM - Spinta media, 30 Hz')
xlabel('step PWM')
ylabel('Spinta media')

%%%%%%%%%%%%%%%%%%%%%%
% 122.55 Hz
a=zeros(6,1); % indice 
for j=1:6
for i=1:length(r122)
    if  r122(i,1)==j-1
        a(j)=a(j)+1;
        k = a(j);
        r122_mat(k,j) = r122(i,2);
    end
end
end

for j=1:6
    r122_step = r122_mat(:,j);
    r122_step(r122_step==0)=[];   % toglie gli zeri dovuti alla presa dati
    r122_value(j) = mean(r122_step)/1e6;
end

% % retta di regressione  --> dati molto sporchi
% xi122 = 4:1:16; % togliamo dove non sia apre
% x_media122 = mean(xi122);
% yi122 = r122_value(4:1:16);
% y_media122 = mean(yi122);
% 
% scarti_x_vect122 = (xi122 - x_media122*ones(1,length(xi122)));
% scarti2_x_vect122 = scarti_x_vect122.^2;
% S_xx0 = 0;
% for i = 1:length(scarti2_x_vect122)
% Sxx = scarti2_x_vect122(i) + S_xx0;  % calcolo della varianza
% S_xx0 = Sxx;
% end
% 
% scarti_y_vect = (yi122 - y_media122*ones(1,length(yi122)));
% S_xy0 = 0;
% for i = 1:length(scarti2_x_vect122)
% Sxy = scarti_x_vect122(i)*scarti_y_vect(i) + S_xy0; % calcolo della covarianza
% S_xy0 = Sxy;
% end
% 
% m122 = Sxy/Sxx;
% q122 = y_media122 - m122*x_media122;
% xx122 = linspace(4, 16, 100);
% yy122 = q122 + m122*xx122;
% 
% plotting
xp122 = 1:1:6;
yi = 9.81*massa_equivalente(r122_value);
% yy122 = 9.81*massa_equivalente(yy122);

figure
plot(xp122,yi,'*')
hold on
% plot(xx122, yy122) % retta di regressione
plot(xp122,yi)
title('Grafico step PWM - Spinta media, 122.55 Hz')
xlabel('step PWM')
ylabel('Spinta media')

%%%%%%%%%%%%%
% 245.10 Hz
a=zeros(4,1); % indice 
for j=1:4
for i=1:length(r245)
    if  r245(i,1)==j-1
        a(j)=a(j)+1;
        k = a(j);
        r245_mat(k,j) = r245(i,2);
    end
end
end

for j=1:4
    r245_step = r245_mat(:,j);
    r245_step(r245_step==0)=[];   % toglie gli zeri dovuti alla presa dati
    r245_value(j) = mean(r245_step)/1e6;
end

% retta di regressione
xi245 = 2:1:4; % togliamo dove non sia apre
x_media245 = mean(xi245);
yi245 = r245_value(2:1:4);
y_media245 = mean(yi245);

scarti_x_vect245 = (xi245 - x_media245*ones(1,length(xi245)));
scarti2_x_vect245 = scarti_x_vect245.^2;
S_xx0 = 0;
for i = 1:length(scarti2_x_vect245)
Sxx = scarti2_x_vect245(i) + S_xx0;  % calcolo della varianza
S_xx0 = Sxx;
end

scarti_y_vect = (yi245 - y_media245*ones(1,length(yi245)));
S_xy0 = 0;
for i = 1:length(scarti2_x_vect245)
Sxy = scarti_x_vect245(i)*scarti_y_vect(i) + S_xy0; % calcolo della covarianza
S_xy0 = Sxy;
end

m245 = Sxy/Sxx;
q245 = y_media245 - m245*x_media245;
xx245 = linspace(2, 4, 100);
yy245 = q245 + m245*xx245;

% plotting
xp245 = 1:1:4;
yi = 9.81*massa_equivalente(r245_value);
yy245 = 9.81*massa_equivalente(yy245);

figure
plot(xp245,yi,'*')
hold on
plot(xx245, yy245)
plot(xp245,yi)
title('Grafico step PWM - Spinta media, 245.10 Hz')
xlabel('step PWM')
ylabel('Spinta media')
%% 
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % %
% % %                    ANALISI DATI
% 
% % % Studio Spinta media
% % S_4_1 = 9.81 * massa_equivalente(r4_1);
% % S_4_2 = 9.81 * massa_equivalente(r4_2);
% % S_4_3 = 9.81 * massa_equivalente(r4_3);
% % S_4_4 = 9.81 * massa_equivalente(r4_4);
% % S_4_5 = 9.81 * massa_equivalente(r4_5);
% % S_4_6 = 9.81 * massa_equivalente(r4_6);
% % S_4_7 = 9.81 * massa_equivalente(r4_7);
% % S_4_8 = 9.81 * massa_equivalente(r4_8);
% % S4 = [S_4_1   S_4_2   S_4_3   S_4_4   S_4_5   S_4_6   S_4_7    S_4_8];
% 
% 
% % Creazione retta di regressione con met sdqm
% xi = 1:1:8;  
% yi = S4;
% % nel caso vedi che i primi punti sono nulli cambia con:
% % xi = inizio_punti_non_nulli:1:8
% % yi = S4' (devi togliere anche da S4)
% 
% 
% x_media = mean(xi);
% y_media = mean(yi);
% 
% scarti_x_vect = (xi - x_media*ones(1,length(xi)));
% scarti2_x_vect = scarti_x_vect.^2;
% S_xx0 = 0;
% 
% for i = 1:length(scarti2_x_vect)
% Sxx = scarti2_x_vect(i) + S_xx0;  % calcolo della varianza
% S_xx0 = Sxx;
% end
% 
% scarti_y_vect = (yi - y_media*ones(1,length(yi)));
% S_xy0 = 0;
% for i = 1:length(scarti2_x_vect)
% Sxy = scarti_x_vect(i)*scarti_y_vect(i) + S_xy0; % calcolo della covarianza
% S_xy0 = Sxy;
% end
% 
% m = Sxy/Sxx;
% q = y_media - m*x_media;
% xx = linspace(0, 8, 160);
% yy = q + m*xx;
% 
% spinta_rettareg = @(nbit) q + m*nbit; 
% 
% % plotting
% figure
% plot(xi,S4,'*')
% hold on
% plot(xx,yy)
% legend('punti sperimentali', 'retta di regressione')
% title('retta di regressione della Spinta media, 4 Hz')
% xlabel('nbit di accensione in T_sample')
% ylabel('Spinta media in T sample')

% % Studio vettore di Steady State, questo rappresenta il piano di 
% %   stabilizzazione della spinta, deve essere il più simile possibile tra
% %   tutte le frequenze perchè dipende solo dalla pressione
% for i=1: length(r4_1_vect)
% if r4_1_vect(i,2)==0 
% break
% 
% else
%     run4_1_SSvect(i)=run4_1(i,3); %vettore di Steady State 
% end
% end
% 
% r4_1_SS = mean(run4_1_SSvect);
% S_4_1max = 9.81*(massa_equivalente(r4_1_SS));
% % Duplica da 1 a 8


% 
% % Poi duplica per tutte le fequenze questa analisi dati e poi compara i
% %   grafici, dovrebbero essere uguali o con un errore minimo insomma, 
% %   perchè la spinta dipende solo dalla pressione. Tolte le frequenze
% %   troppo alte


% %%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %
% %           SECONDO ESPERIMENTO
% %
% %               DATI
% %   - Frequenza di PWM variabile (valori alti)
% %   - Testiamo 2,3,6,8,16,24,32 ms
% %   - Pressione regolatore 30 psi
% %   - Frequenza amplificatore 80 Hz
% %   - Secondi acquisizione 0.5s --> 4s in tot
% %   - 8 bit di precisione del PWM
% %   - n bit di acquisizione preimpostati = 8 (acquisisce per tutti i 
% %     livelli di precisione)
% %  
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %
% %                           PRESA DATI
% %
% 
% % divisione in casi a seconda del minimo intervallo di tempo
% % t_on = T_sample/8
% 
% %%%%%%%%%%%%%% 2 ms
% load('Esp2_Trisposta/r1_2ms.txt')
% % load('Esp2_Trisposta/r2_2ms.txt')
% % load('Esp2_Trisposta/r3_2ms.txt')
% 
% r1_2ms = mean(r1_2ms(10:length(r1_2ms)-10))/1e6;   %ha meno dati questo e quello da 4
% % r2_2ms = mean(r2_2ms(50:length(r2_2ms)-50))/1e6;
% % r3_2ms = mean(r3_2ms(50:length(r3_2ms)-50))/1e6;
% 
% % r_2ms = mean([r1_2ms, r2_2ms, r3_2ms]);
% 
% %%%%%%%%%%%% 4 ms
% load('Esp2_Trisposta/r1_4ms.txt')
% % load('Esp2_Trisposta/r2_4ms.txt')
% % load('Esp2_Trisposta/r3_4ms.txt')
% 
% r1_4ms = mean(r1_4ms(10:length(r1_4ms)-10))/1e6;
% % r2_4ms = mean(r2_4ms(50:length(r2_4ms)-50))/1e6;
% % r3_4ms = mean(r3_4ms(50:length(r3_4ms)-50))/1e6;
% % 
% % r_4ms = mean([r1_4ms, r2_4ms, r3_4ms]);
% 
% 
% %%%%%%%%%%%% 6 ms
% load('Esp2_Trisposta/r1_6ms.txt')
% % load('Esp2_Trisposta/r2_6ms.txt')
% % load('Esp2_Trisposta/r3_6ms.txt')
% 
% r1_6ms = mean(r1_6ms(50:length(r1_6ms)-50))/1e6;
% % r2_6ms = mean(r2_6ms(50:length(r2_6ms)-50))/1e6;
% % r3_6ms = mean(r3_6ms(50:length(r3_6ms)-50))/1e6;
% % 
% % r_6ms = mean([r1_6ms, r2_6ms, r3_6ms]);
% 
% 
% %%%%%%%%%%%% 8 ms
% load('Esp2_Trisposta/r1_8ms.txt')
% % load('Esp2_Trisposta/r2_8ms.txt')
% % load('Esp2_Trisposta/r3_8ms.txt')
% 
% r1_8ms = mean(r1_8ms(50:length(r1_8ms)-50))/1e6;
% % r2_8ms = mean(r2_8ms(50:length(r2_8ms)-50))/1e6;
% % r3_8ms = mean(r3_8ms(50:length(r3_8ms)-50))/1e6;
% % 
% % r_8ms = mean([r1_8ms, r2_8ms, r3_8ms]);
% 
% %%%%%%%%%%%% 10 ms
% load('Esp2_Trisposta/r1_10ms.txt')
% % load('Esp2_Trisposta/r2_8ms.txt')
% % load('Esp2_Trisposta/r3_8ms.txt')
% 
% r1_10ms = mean(r1_10ms(50:length(r1_10ms)-50))/1e6;
% % r2_8ms = mean(r2_8ms(50:length(r2_8ms)-50))/1e6;
% % r3_8ms = mean(r3_8ms(50:length(r3_8ms)-50))/1e6;
% % 
% % r_8ms = mean([r1_8ms, r2_8ms, r3_8ms]);
% 
% 
% %%%%%%%%%%%% 12 ms
% load('Esp2_Trisposta/r1_12ms.txt')
% % load('Esp2_Trisposta/r2_8ms.txt')
% % load('Esp2_Trisposta/r3_8ms.txt')
% 
% r1_12ms = mean(r1_12ms(50:length(r1_12ms)-50))/1e6;
% % r2_8ms = mean(r2_8ms(50:length(r2_8ms)-50))/1e6;
% % r3_8ms = mean(r3_8ms(50:length(r3_8ms)-50))/1e6;
% % 
% % r_8ms = mean([r1_8ms, r2_8ms, r3_8ms]);
% 
% 
% %%%%%%%%%%%% 14 ms
% load('Esp2_Trisposta/r1_14ms.txt')
% % load('Esp2_Trisposta/r2_8ms.txt')
% % load('Esp2_Trisposta/r3_8ms.txt')
% 
% r1_14ms = mean(r1_14ms(50:length(r1_14ms)-50))/1e6;
% % r2_8ms = mean(r2_8ms(50:length(r2_8ms)-50))/1e6;
% % r3_8ms = mean(r3_8ms(50:length(r3_8ms)-50))/1e6;
% % 
% % r_8ms = mean([r1_8ms, r2_8ms, r3_8ms]);
% 
% %%%%%%%%%%%% 16 ms
% load('Esp2_Trisposta/r1_16ms.txt')
% % load('Esp2_Trisposta/r2_16ms.txt')
% % load('Esp2_Trisposta/r3_16ms.txt')
% 
% r1_16ms = mean(r1_16ms(50:length(r1_16ms)-50))/1e6;
% % r2_16ms = mean(r2_16ms(50:length(r2_16ms)-50))/1e6;
% % r3_16ms = mean(r3_16ms(50:length(r3_16ms)-50))/1e6;
% % 
% % r_16ms = mean([r1_16ms, r2_16ms, r3_16ms]);
% 
% 
% %%%%%%%%%%% 24 ms     SICURI SIA UTILE???????????
% load('Esp2_Trisposta/r1_24ms.txt')
% % load('Esp2_Trisposta/r2_24ms.txt')
% % load('Esp2_Trisposta/r3_24ms.txt')
% 
% r1_24ms = mean(r1_24ms(50:length(r1_24ms)-50))/1e6;
% % r2_24ms = mean(r2_24ms(50:length(r2_24ms)-50))/1e6;
% % r3_24ms = mean(r3_24ms(50:length(r3_24ms)-50))/1e6;
% % 
% % r_24ms = mean([r1_24ms, r2_24ms, r3_24ms]);
% 
% 
% %%%%%%%%%%% 32 ms     SICURI SIA UTILE???????????
% load('Esp2_Trisposta/r1_32ms.txt')
% % load('Esp2_Trisposta/r2_32ms.txt')
% % load('Esp2_Trisposta/r3_32ms.txt')
% 
% r1_32ms = mean(r1_32ms(50:length(r1_32ms)-50))/1e6;
% % r2_32ms = mean(r2_32ms(50:length(r2_32ms)-50))/1e6;
% % r3_32ms = mean(r3_32ms(50:length(r3_32ms)-50))/1e6;
% % 
% % r_32ms = mean([r1_32ms, r2_32ms, r3_32ms]);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%
% 
% 
% % 
% % xx2 = [2 4 6 8 16 24 32];
% % yy2 = [r_2ms r_4ms r_6ms r_8ms r_16ms r_24ms r_32ms];
% 
% xx2 = [2 4 6 8 10 12 14 16 24 32];
% yy2 = [r1_2ms r1_4ms r1_6ms r1_8ms r1_10ms r1_12ms r1_14ms r1_16ms r1_24ms r1_32ms];
% S_2 = 9.81*massa_equivalente(yy2);
% % lo studio di questa la facciamo alla fine di tutto perchè è da creare un
% % limite di spinta oltre la quale riteniamo che non si sia aperta del tutto
% % Intanto raccogliete dati e plottate, poi vediamo in seguito
% 
% % plotting
% plot(xx2,S_2)
% legend('Punti sperimentali')
% title('Caduta della Spinta media - Ricerca del T di ON&OFF')
% xlabel('Minimo tempo di accensione [ms]')
% ylabel('Spinta media [mN]')



%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %
% %          TERZO ESPERIMENTO
% %
% %               DATI
% %   - Frequenza di PWM variabile (valori alti)
% %   - Pressione regolatore 30 psi
% %   - Frequenza amplificatore 80 Hz
% %   - Secondi acquisizione 0.5s --> 4s in tot
% %   - 8 bit di precisione del PWM
% %   - n bit di acquisizione preimpostati
% %  
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %
% %                           PRESA DATI
% 
% % carica le run
% load('run1.txt')
% load('run2.txt')
% load('run3.txt')
% 
% % non so come siano fatti i dati --> supponiamo siano un treno di dati così
% % [tempo; lettura della elettrovalvola]
% 
% xx_1 = run1(:,1); 
% yy_1 = run1(:,2);
% 
% xx_2 = run2(:,1); 
% yy_2 = run2(:,2);
% 
% xx_3 = run3(:,1); 
% yy_3 = run3(:,2);
% 
% % verranno sicuramente molto sporchi, pieni di rumore quindi vanno
% % interpolati con una funziona appropriata, a occhio non sembreranno avere
% % senso
% 
% % la mia idea è quella di creare 
% 
% % plotting
% plot(xx3_1,yy3_1)
% hold on
% plot(xx3_2,yy3_2)
% plot(xx3_3,yy3_3)
% legend('punti sperimentali')
% title('Caduta della Spinta media')
% xlabel('Frequenze PWM')
% ylabel('Spinta media')