%% ======================= MODEL PREDICTIVE CONTROL =======================
% ============================= DOUBLE LINK ============================= 

clc; clear all; close all;

%% ================= DMD REDUCED MATRICES del double-link =================   
if ~isfile('dynamics_reduced_dim.mat')
    error('File dynamics_reduced_dim.mat non trovato nella cartella corrente');
end

load('dynamics_reduced_dim.mat'); % deve contenere: Uhat, approxA, approxB

% 1. CORREZIONE DEL GUADAGNO (Scaling per dare forza)
approxB = approxB * 500; 

% 2. CORREZIONE DEL SEGNO (Inversione per la direzione)
approxB(:, 2) = -approxB(:, 2);

%disp(['Norma della matrice B (approxB): ', num2str(norm(approxB))]);
%disp(['Norma B colonna 1: ', num2str(norm(B(:, 1)))]);
%disp(['Norma B colonna 2: ', num2str(norm(B(:, 2)))]);

%% ========================= PHYSICAL PARATEMERS =========================
global g m1 m2 l1 l2 v1 v2 height width n_states n_controls xref

height = 100;
width  = 100;

% Parametri fisici
g  = 9.81;            % [m/s^2]
l1 = 1.0; l2 = 0.7;   % [m]     
m1 = 2.0; m2 = 1.5    % [kg]
v1 = 6.0; v2 = 3.0    % [kgms]

n_states    = 4;   % [q1; q2; dq1; dq2]
n_controls  = 2;   % [u1; u2]

%% ========== STATI INIZIALI E RIFERIMENTO (DOMINIO FISICO) ==========
% stato fisico
x_initial = [0; 0; 0; 0];        % [q1; q2; dq1; dq2], angolo e velocità iniziali
xref      = [pi/4; pi/6; 0; 0];  % riferimento in spazio stati, angolo e velocità desiderati

% immagini iniziale e riferimento (per dominio DMD)
x_ini_path = './x_ini_double_link.png';
x_ref_path = './x_ref_double_link.png';

x_ini_img = img2array(x_ini_path);   % 100x100
x_ref_img = img2array(x_ref_path);   % 100x100

% Proiezione nello spazio ridotto (dimensione r = size(Uhat,2))
x_ini = Uhat' * x_ini_img(:);        % r x 1 -> 30 x 1
x_ref = Uhat' * x_ref_img(:);        % r x 1 -> 30 x 1

%% ========== HYPERPARAMETRI MPC (DOMINIO RIDOTTO DMD) ==========
A  = approxA;
B  = approxB;

nx = size(A,2);        % dimensione stato ridotto (30)
nu = size(B,2);        % dimensione input ridotto (dovrebbe essere 2)

Q = 500 * eye(nx);     % prima era 100
R = 0.1  * eye(nu);    % prima era 10  

Np = 10;               % prediction horizon, prima era 5

% vincoli sugli input
Hu = [ eye(nu); -eye(nu)];
bu = 20 * ones(2*nu,1);  % prima era 5, -5 <= u_i <= 5 

%% ========== COSTRUZIONE OPTIMIZER (YALMIP + GUROBI) ==========
u  = sdpvar(nu, Np);
x  = sdpvar(nx, Np+1);
x0 = sdpvar(nx, 1);     % stato ridotto iniziale
xr = sdpvar(nx, 1);     % stato ridotto di riferimento

constraints = [];
objective   = 0;

% vincolo stato iniziale
constraints = [constraints, x(:,1) == x0];

for k = 1:Np
    % costo di stadio
    objective = objective + (x(:,k) - xr)'*Q*(x(:,k) - xr) + u(:,k)'*R*u(:,k);

    % dinamica ridotta
    constraints = [constraints, x(:,k+1) == A*x(:,k) + B*u(:,k)];

    % vincolo sugli input
    constraints = [constraints, Hu*u(:,k) <= bu];
end

% opzionale: costo terminale (volendo puoi riattivarlo)
% objective = objective + (x(:,Np+1) - xr)'*Q*(x(:,Np+1) - xr);

ops = sdpsettings('verbose', 2, 'solver','gurobi', 'gurobi.Presolve', 0);

predictor = optimizer(constraints, objective, ops, {x0, xr}, {x, u});

%% ========== SIMULAZIONE CLOSED-LOOP ==========
dt_step = 0.01;
tspan = [0 dt_step];     % passo integrazione dinamica reale
t_final = 3.0;           % [s], prima era 10

xh    = x_initial;       % storico stati fisici (4 x N)
x_dmd = x_ini;           % storico stati ridotti (r x N)
uh    = [];              % storico controlli (2 x N)

filename = 'animation_double_link.gif';

figure(1); clf;
set(gcf, 'Color', 'w');

i_step = 0;
for t = 0: dt_step: t_final
    i_step = i_step + 1;

    % === MPC nel dominio ridotto ===
    sol    = predictor(x_dmd(:,end), x_ref);
    x_pred = sol{1};
    u_pred = sol{2};

    u_curr = u_pred(:,1);         % primo ingresso ottimo
    uh     = [uh, u_curr];

    % === Dinamica nel dominio fisico (4 stati) ===
    [~, x_ode] = ode45(@(tt,xx) dynamics2link(tt, xx, u_curr), tspan, xh(:,end));  %[t_ode, x_ode]
    xh = [xh, x_ode(end,:)'];     % aggiungo ultimo stato simulato

    % === Generazione immagine e GIF ===
    cla;
    img_generation(xh(1:2, end), '.');   % creazione frame_0001.png 
    %path = img_generation(xh(:,end));   % salva ./current_img.png

    % === Rinomino in current_img.ong ===
    if exist('current_img.png', 'file')
        delete('current_img.png');  % cancello eventuale file precedente
    end
    movefile('frame_0001.png', 'current_img.png');

    % === Percorso immagine finale ===
    path = './current_img.png';

    drawnow;
    frame = getframe(gcf);
    im    = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);

    if i_step == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', 1, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end

    % === Aggiorno stato ridotto DMD a partire dall'immagine corrente ===
    x_img  = img2array(path);          % 100x100
    x_dmd  = [x_dmd, Uhat'*x_img(:)];  % r x 1
end

%% ========== PLOT RISULTATI NUMERICI ==========
% vettore tempi coerente con la simulazione
N = size(xh,2);
t_vec = linspace(0, t_final, N);

figure(2); clf;

% q1
subplot(4,1,1)
plot(t_vec, rad2deg(xh(1,:)), 'Color','#0072BD','LineWidth',2); hold on;
yline(rad2deg(xref(1)), '--r','LineWidth',1.5);
ylabel('\theta_1 [deg]');
grid on;
xlim([0 t_final]);

% q2
subplot(4,1,2)
plot(t_vec, rad2deg(xh(2,:)), 'Color','#D95319','LineWidth',2); hold on;
yline(rad2deg(xref(2)), '--r','LineWidth',1.5);
ylabel('\theta_2 [deg]');
grid on;
xlim([0 t_final]);

% velocità
subplot(4,1,3)
plot(t_vec, xh(3,:), 'LineWidth',2); hold on;
plot(t_vec, xh(4,:), 'LineWidth',2);
yline(0,'--k','LineWidth',1);
ylabel('velocities [rad/s]');
legend('\omega_1','\omega_2','Location','best');
grid on;
xlim([0 t_final]);

% controlli
subplot(4,1,4)
if ~isempty(uh)
    t_u = t_vec(1:size(uh,2));  % stessa lunghezza di uh
    stairs(t_u, uh(1,:), 'LineWidth',2); hold on;
    stairs(t_u, uh(2,:), 'LineWidth',2);
    yline(max(bu),  '--','LineWidth',1.5);
    yline(-max(bu), '--','LineWidth',1.5);
end

xlabel('Tempo [s]');
ylabel('u_1, u_2');
legend('u_1','u_2','u_{max}','u_{min}','Location','best');
grid on;
xlim([0 t_final]);

sgtitle('MPC modello ridotto');
saveas(gcf, 'MPC modello ridotto.png');

%% ===================== FUNZIONI DI SUPPORTO ============================

function array = img2array(path)
    global height width
    img = imread(path);
    img = imresize(img, [height width]);

    if size(img,3) == 3
        gray_img = rgb2gray(img);
    else
        gray_img = img;
    end

    array = double(gray_img) / 255.0;
end

function final_state = rk4_simulation(x0, dynamics_fun, u)
    global n_states
    dt = 0.01;
    tf = 0.1;

    time  = 0:dt:tf;
    state = zeros(n_states, length(time));
    state(:,1) = x0;

    for q = 1:length(time)-1
        k1 = dynamics_fun(time(q),           state(:,q),           u);
        k2 = dynamics_fun(time(q) + dt/2.0,  state(:,q) + dt/2.0 * k1, u);
        k3 = dynamics_fun(time(q) + dt/2.0,  state(:,q) + dt/2.0 * k2, u);
        k4 = dynamics_fun(time(q) + dt,      state(:,q) + dt      * k3, u);

        state(:,q+1) = state(:,q) + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4);
    end

    final_state = state(:,end);
end

%function dx = dynamics(~, x, u)
    % Dinamica di un robot planare a 2 link
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
    %M = [M11 M12;
         %M21 M22];

    % --- Coriolis/Centrifuga C(q,dq) ---
    %C11 = -m2*l1*l2*sin(q2)*dq2;
    %C12 = -m2*l1*l2*sin(q2)*(dq1 + dq2);
    %C21 =  m2*l1*l2*sin(q2)*dq1;
    %C22 =  0;
    %C = [C11 C12;
         %C21 C22];

    % --- Gravità G(q) ---
    %G1 = (m1+m2)*g*l1*sin(q1) + m2*g*l2*sin(q1 + q2);
    %G2 = m2*g*l2*sin(q1 + q2);
    %G = [G1; G2];

    % --- Attrito viscoso D dq ---
    %D = [v1 0;
         %0  v2];

    % u = [u1; u2]
    %ddq = M \ (u - C*dq - D*dq - G);

    %dx = [dq;
          %ddq];
%end

%function save_path = img_generation(state_t)
    %global l1 l2

    %q1 = state_t(1);
    %q2 = state_t(2);

    % Primo link (base -> giunto 1)
    %x1 = l1 * cos(q1);
    %y1 = l1 * sin(q1);

    % Secondo link (giunto 1 -> giunto 2)
    %x2 = x1 + l2 * cos(q1 + q2);
    %y2 = y1 + l2 * sin(q1 + q2);

    % Disegno
    %plot([0 x1], [0 y1], 'LineWidth',8,'Color','k'); hold on;
    %plot([x1 x2], [y1 y2], 'LineWidth',8,'Color','k');

    %plot(x1, y1, 'ko','MarkerSize',8,'MarkerFaceColor','k');
    %plot(x2, y2, 'ko','MarkerSize',8,'MarkerFaceColor','k');

    %axis equal;
    %xlim([- (l1+l2) (l1+l2)]);
    %ylim([- (l1+l2) (l1+l2)]);

    %set(gca, 'XColor','none', 'YColor','none');
    %set(gca, 'TickLength',[0 0]);
    %set(gcf, 'Color','w');

    %save_path = './current_img.png';
    %saveas(gcf, save_path);

    %hold off;
%end