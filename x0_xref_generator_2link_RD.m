%% ============ Double rigid link robotic arm ============

% Pulizia Command Window, cancellazione di tutte le variabili
% e funzioni dalla memoria e chiusura di tutte le figure aperte
clc; clear; close all;

tic 

%% ================= Parametri del sistema =================
params.l1 = 1;       % lunghezza link 1 [m]
params.l2 = 0.7;     % lunghezza link 2 [m]
params.m1 = 2;       % massa link 1 [kg]
params.m2 = 1.5;     % massa link 2 [kg]
params.v1 = 6;       % smorzamento link 1 [kg*m*s]
params.v2 = 3;       % smorzamento link 2 [kg*m*s]
params.g  = 9.81;    % gravità [m/s^2]

%% ================= Stato iniziale =================
x0 = [0; 0; 0; 0];  % [theta1; theta2; dtheta1; dtheta2]

%% ================= Stato di riferimento =================
x_ref = [pi/4; pi/6; 0; 0];  % posizione target dei due link

%% ================= Disegno iniziale/finale =================
draw_robotic_arm(x_ref, params); % FORNIRE ANCHE x0

%% ================= Funzione principale =================
function draw_robotic_arm(x, params)
    figure;
    % axis equal; 
    % grid on; 
    hold on;
    % set(gca, 'Position', [0 0 1 1]);  % rimuove margini
    set(gca, 'XColor', 'none', 'YColor', 'none');% nasconde assi
    set(gca, 'XTick', [], 'YTick', []); % rimuove i numeri dagli assi
    xlim([-2 2]); ylim([-2 2]);
    
    % Crea le linee dei link
    link1 = line([0 0], [0 0], 'LineWidth', 8, 'Color', 'k');
    link2 = line([0 0], [0 0], 'LineWidth', 8, 'Color', 'k');
    
    % Estrai angoli
    theta1 = x(1);
    theta2 = x(2);
    
    % Calcola coordinate
    x1 = params.l1 * cos(theta1);
    y1 = params.l1 * sin(theta1);
    
    x2 = x1 + params.l2 * cos(theta1 + theta2);
    y2 = y1 + params.l2 * sin(theta1 + theta2);
    
    % Aggiorna le linee
    set(link1, 'XData', [0 x1], 'YData', [0 y1]);
    set(link2, 'XData', [x1 x2], 'YData', [y1 y2]);
    
    % Disegna giunti
    plot(0,0,'ko','MarkerSize',8,'MarkerFaceColor','k'); % base
    plot(x1, y1, 'ko', 'MarkerSize',8, 'MarkerFaceColor','k'); % giunto intermedio
    plot(x2, y2, 'ko', 'MarkerSize',8, 'MarkerFaceColor','k'); % estremità
    
    % Sfondo bianco
    set(gcf, 'Color', 'w');
    set(gca, 'Color', 'none');
    
    % Salva immagine
    saveas(gcf, './x_ref_double_link.png');
end