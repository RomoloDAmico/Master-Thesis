% ===========================================================
% DOUBLE RIGID LINK ROBOTIC ARM - DATASET GENERATOR (RK4)
% ===========================================================

clc; clear all; close all;

tic % Avvio timer

%% ================== CREAZIONE CARTELLA PRINCIPALE ==================
base_folder = './Simulazioni';
if ~exist(base_folder, 'dir')
    mkdir(base_folder);
end

%% ================== PARAMETRI DEL SISTEMA ==================
global g l1 l2 m1 m2 v1 v2

% Parametri
g = 9.81; % accelerazione di gravità [m/s^2]
l1 = 1; l2 = 0.7; % lunghezze dei link
m1 = 2; m2 = 1.5; % masse dei link
v1 = 6; v2 = 3; % attriti

%% ================== GENERAZIONE Nsim ==================
Nsim = 50; % numero di simulazioni "random"

for sim = 1:Nsim
    % Crea cartella per la simulazione
    sim_folder = sprintf('%s/sim%d', base_folder, sim);
    if ~exist(sim_folder, 'dir')
        mkdir(sim_folder);
    end

    % Initialize simulation parameters
    fprintf('Starting simulation %d of %d...\n', sim, Nsim);

    % Condizioni iniziali casuali
    % angoli iniziali tra -π e π --> copro tutta la rotazione possibile del braccio 
    % velocità iniziali tra -2 e 2 rad/s --> traiettorie più veloci e variegate
    % x0 = [ ...
        % -pi/2 + pi * rand();  % theta1 oppure scrivere randn
        % +1 + 2 * rand();      % dtheta1
        % -pi/2 * pi * rand();  % theta2
        % -1 + 2 * rand()];     % dtheta2
    % Condizioni iniziali casuali corrette
    theta0 = -pi + 2*pi*rand(2,1);    % theta1 e theta2 ∈ [-π, π]
    dtheta0 = -2 + 4*rand(2,1);       % dtheta1 e dtheta2 ∈ [-2, 2] rad/s
    x0 = [theta0(1); theta0(2); dtheta0(1); dtheta0(2)];  % 4x1


    % Intervallo di integrazione
    t0 = 0; tf = 3; dt = 0.01;
    t = t0:dt:tf;
    x = zeros(4, length(t));
    x(:,1) = x0;
    u = zeros(2, length(t) - 1); % inizializzo la matrice dove salvare ii controlli a ogni instante

    % Integrazione RK4
    for i = 1:length(t) - 1  % prima era i = 1:Nsim:length(t) 
        % cla;
        u(:,i) = -20 + 40*rand(2,1); % controlli casuali
        k1 = dynamics2link(t(i), x(:,i), u(:,i));
        k2 = dynamics2link(t(i)+dt/2, x(:,i)+dt/2*k1, u(:,i));
        k3 = dynamics2link(t(i)+dt/2, x(:,i)+dt/2*k2, u(:,i));
        k4 = dynamics2link(t(i)+dt, x(:,i)+dt*k3, u(:,i));
        x(:,i+1) = x(:,i) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
        % filename = sprintf('%s/frame_%04d.png', sim_folder, i);
        % saveas(gcf, filename)
    end

    % Creazione cartella
    %folder = sprintf('./dataset/sim%d', sim);
    %if ~exist(folder, 'dir')
        %mkdir(folder);
    %end
    
    % Salvataggio dati numerici
    save(fullfile(sim_folder, 'sim_data.mat'), 't', 'u', 'x');

    img_generation(x, sim_folder);

    % Animazione e salvataggio frame
    %figure(1);
    %clf;
    %axis equal;
    %xlim([- (l1 + l2), (l1 + l2)]);
    %ylim([- (l1 + l2), (l1 + l2)]);
    %set(gca, 'XColor', 'none', 'YColor', 'none');
    %set(gcf, 'Color', 'w');
    %set(gca, 'Color', 'none');
    %hold on;

end

fprintf('\n✅ Sim1–Sim%d generate con successo!\n', Nsim);

%% ================== CONDIZIONI INIZIALI SIMULAZIONE FINALE ==================
fprintf('\n==== Simulazione finale: SimFinale ====\n')

x0    = [0; 0; 0; 0]
x_ref = [pi/4; pi/6; 0; 0]; % [theta1, theta2, dtheta1, dtheta2]

% prima avevo x0 = [pi/4; pi/6; 0; 0];

%% ================= INTERVALLO DI INTEGRAZIONE =================
t0 = 0; tf = 3; dt = 0.01;
t = t0:dt:tf;
x = zeros(4, length(t));
x(:,1) = x0;
u = zeros(2, length(t)-1); % 2 controlli

%% ========================== INTEGRAZIONE RK4 ==========================
for i = 1:length(t)-1
    u(:,i) = -20 + 40*rand(2,1); % controlli casuali
    k1 = dynamics2link(t(i), x(:,i), u(:,i));
    k2 = dynamics2link(t(i)+dt/2, x(:,i)+dt/2*k1, u(:,i));
    k3 = dynamics2link(t(i)+dt/2, x(:,i)+dt/2*k2, u(:,i));
    k4 = dynamics2link(t(i)+dt, x(:,i)+dt*k3, u(:,i));
    x(:,i+1) = x(:,i) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

finale_folder = fullfile(base_folder, 'simFinale');
if ~exist(finale_folder, 'dir')
    mkdir(finale_folder);
end

% salvataggio dati numerici
save(fullfile(finale_folder, 'sim_data.mat'), 't','u','x');

img_generation(x, finale_folder);

%% ====== STAMPA DIMENSIONI E PRIMI VALORI PER CONTROLLARE ====== 
size(x)    % deve essere 4 x N
disp('prime colonne di x:')
disp(x(:,1:5))

%% ============== CONTROLLA QUALCHE COPPIA DI ANGOLI ==============
q1 = x(1,1); q2 = x(2,1);
fprintf('q1(1)=%.3f, q2(1)=%.3f\n', q1, q2);

%% ================== ANIMAZIONE ==================
%for i = 1:10:length(t)
    %cla;
    %q1 = x(1,i); q2 = x(3,i);
    %P1 = [l1*sin(q1), -l1*cos(q1)];
    %P2 = P1 + [l2*sin(q1+q2), -l2*cos(q1+q2)];
    %line([0 P1(1)], [0 P1(2)], 'LineWidth', 4, 'Color', 'k');
    %line([P1(1) P2(1)], [P1(2) P2(2)], 'LineWidth', 4, 'Color', 'r');
    %plot(P2(1), P2(2), 'ko','MarkerFaceColor','k');
    %axis equal; xlim([-2 2]); ylim([-2 2]);
    %drawnow;
%end

% params l1, l2 definiti precedentemente
%figure(1); 
%clf;
%axis equal;
%grid on;
%xlim([- (l1 + l2) - 0.2, (l1 + l2) + 0.2]);
%ylim([- (l1 + l2) - 0.2, (l1 + l2) + 0.2]);
%set(gca,'XColor','none','YColor','none');
%set(gcf, 'Color', 'w'); % sfondo bianco
%set(gca, 'Color', 'none');
%hold on;

%% ================ DINAMICA DEL DOPPIO LINK ================
%function dx = dynamics2link(~, x, u)
    %global g l1 l2 m1 m2 v1 v2

    % Stati
    %q1 = x(1); 
    %q2 = x(2);
    %dq1 = x(3);
    %dq2 = x(4);
    %dq = [dq1; dq2];

    % Matrici del modello (semplificate)
    %M11 = (m1+m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(q2);
    %M12 = m2*l2^2 + m2*l1*l2*cos(q2);
    %M21 = M12;
    %M22 = m2*l2^2;
    %M = [M11 M12; M21 M22];

    %C1 = -m2*l1*l2*(2*dq1*dq2 + dq2^2)*sin(q2);
    %C2 =  m2*l1*l2*dq1^2*sin(q2);
    %C = [C1; C2];

    %G1 = (m1+m2)*g*l1*sin(q1) + m2*g*l2*sin(q1+q2);
    %G2 = m2*g*l2*sin(q1+q2);
    %G = [G1; G2];

    %tau = u - [v1*dq1; v2*dq2]; % attriti viscosi
    %ddq = M \ (tau - C - G);

    %dx = [dq; ddq];
%end

%function dx = dynamics(~, x, u)
    % Dinamica del robot planare a 2 link (UNIFICATA)
    %global g m1 m2 l1 l2 v1 v2
    %q1  = x(1);
    %q2  = x(2);
    %dq1 = x(3);
    %dq2 = x(4);
    %dq  = [dq1; dq2];
    
    % --- Matrice di inerzia M(q) ---
    %M11 = (m1+m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(q2);
    %M12 = m2*l2^2 + m2*l1*l2*cos(q2);
    %M21 = M12;
    %M22 = m2*l2^2;
    %M = [M11 M12; M21 M22];

    % --- Forze Coriolis/Centrifughe C*dq ---
    %h = -m2*l1*l2*sin(q2); % Termine h per semplificare
    %Cdq1 = h * (2*dq1*dq2 + dq2^2);
    %Cdq2 = -h * dq1^2;
    %Cdq = [Cdq1; Cdq2];
    
    % --- Gravità G(q) (Versione 'data_generator') ---
    %G1 = (m1+m2)*g*l1*sin(q1) + m2*g*l2*sin(q1 + q2);
    %G2 = m2*g*l2*sin(q1 + q2);
    %G = [G1; G2];
    
    % --- Attrito viscoso D dq ---
    %Ddq = [v1*dq1; v2*dq2];
    
    % --- Equazione del moto ---
    %ddq = M \ (u - Cdq - Ddq - G);
    %dx = [dq; ddq];
%end

elapsed_time = toc;  % ⏱️ Ferma timer e restituisce il tempo in secondi
fprintf('⏳ Durata totale esecuzione: %.2f secondi\n', elapsed_time);

minutes = floor(elapsed_time / 60);
seconds = mod(elapsed_time, 60);
fprintf('⏳ Durata totale esecuzione: %d min %.2f s\n', minutes, seconds);
fprintf('\n✅ Tutte le simulazioni sono state salvate in: %s\n', base_folder);