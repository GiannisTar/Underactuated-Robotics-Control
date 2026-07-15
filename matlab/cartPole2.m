% Cart-Pole simulation: Uncontrolled, LQR-Controlled, Swing-Up, and Sliding Mode (SMC) with Friction
clear; clc; close all;

%% Parameters
m_c = 1.0;      % mass of cart (kg)
m_p = 0.1;      % mass of pole (kg)
l = 0.5;        % half-pole length (m)
g = 9.81;       % gravity (m/s^2)
dt = 0.001;     % time step (s)
T = 10;         % total simulation time (s)
N = round(T/dt);

% Friction parameters (from van den Berg et al., ECC 2007, if available)
b_c = 0.5;    % Cart friction coefficient (N*s/m)
b_p = 0.03;   % Pole friction coefficient (N*m*s/rad)

%% State: [x; dx; theta; dtheta]
% θ=0: downward, θ=π: upright
x0 = [0; 0; pi-0.1; 0]; % near upright used for uncontrolled and LQR cases

% Dynamics with friction
cartpole_f = @(x, u) [
    x(2);
    (u - b_c*x(2) + m_p * sin(x(3)) * (l * x(4).^2 + g * cos(x(3)))) / (m_c + m_p * sin(x(3)).^2);
    x(4);
    (-u * cos(x(3)) + b_c*x(2)*cos(x(3)) - m_p * l * x(4).^2 * cos(x(3)) * sin(x(3)) ...
     - (m_c + m_p) * g * sin(x(3)) - b_p*x(4)) / (l * (m_c + m_p * sin(x(3)).^2))
];

%% --- Uncontrolled Simulation (semi-implicit Euler) ---
X_unc = zeros(4, N); X_unc(:,1) = x0; U_unc = zeros(1,N);
for k = 1:N-1
    xk = X_unc(:,k);
    dxk = cartpole_f(xk, 0);
    v_next = xk([2 4]) + dt * dxk([2 4]);
    x_next = xk([1 3]) + dt * v_next;
    xkp1 = [x_next(1); v_next(1); x_next(2); v_next(2)];
    xkp1(3) = wrap_angle(xkp1(3));
    X_unc(:,k+1) = xkp1;
    U_unc(k+1) = 0;
end

%% --- LQR Controller ---
theta_eq = pi;
Xeq = [0;0;theta_eq;0];
ueq = 0;
A = zeros(4,4); B = zeros(4,1); eps = 1e-5;
for i = 1:4
    dx = zeros(4,1); dx(i) = eps;
    A(:,i) = (cartpole_f(Xeq+dx,ueq)-cartpole_f(Xeq-dx,ueq))/(2*eps);
end
B = (cartpole_f(Xeq,ueq+eps)-cartpole_f(Xeq,ueq-eps))/(2*eps);
Q = diag([10 1 100 1]);
R = 0.1;
K = lqr(A,B,Q,R);
umax = 1000;

%% --- LQR-Controlled Simulation (semi-implicit Euler) ---
X_lqr = zeros(4,N); X_lqr(:,1) = x0; U_lqr = zeros(1,N);
for k = 1:N-1
    xk = X_lqr(:,k);
    x_err = xk - [0;0;pi;0];
    x_err(3) = wrap_angle(x_err(3));
    uk = -K*x_err; uk = max(min(uk,umax),-umax);
    dxk = cartpole_f(xk, uk);
    v_next = xk([2 4]) + dt * dxk([2 4]);
    x_next = xk([1 3]) + dt * v_next;
    xkp1 = [x_next(1); v_next(1); x_next(2); v_next(2)];
    xkp1(3) = wrap_angle(xkp1(3));
    X_lqr(:,k+1) = xkp1;
    U_lqr(k+1) = uk;
end

%% --- Swing-Up Controller Simulation (semi-implicit Euler) ---
k_swing = 45;
x0_swingUp = [0; 0; 0.1; -1];
X_swing = zeros(4, N); X_swing(:,1) = x0_swingUp; U_swing = zeros(1,N);

theta_enter_lqr = 0.3;  % Enter LQR when <0.3 rad from π
theta_exit_lqr = 0.5;   % Exit LQR when >0.5 rad from π
in_lqr_mode = false;    % Track LQR state
Xeq = [0; 0; pi; 0];
umax = 50;

Q = diag([200, 20, 100, 10]);
R = 0.1;
K = lqr(A,B,Q,R);

for k = 1:N-1
    xk = X_swing(:,k);
    theta = wrap_angle(xk(3));
    angle_diff = abs(theta - pi);
    angle_diff = min(angle_diff, 2*pi - angle_diff);
    if in_lqr_mode
        if angle_diff > theta_exit_lqr
            in_lqr_mode = false;
        end
    else
        if angle_diff < theta_enter_lqr
            in_lqr_mode = true;
        end
    end
    if in_lqr_mode
        x_err = [xk(1); xk(2); wrap_angle(theta - pi); xk(4)];
        uk = -K*x_err - 1.5*xk(2);
        uk = max(min(uk, umax), -umax);
    else
        uk = swingup_controller(xk, m_p, l, g, umax, k_swing);
        uk = uk - 4*xk(1) - 2*xk(2);
    end
    dxk = cartpole_f(xk, uk);
    v_next = xk([2 4]) + dt * dxk([2 4]);
    x_next = xk([1 3]) + dt * v_next;
    xkp1 = [x_next(1); v_next(1); x_next(2); v_next(2)];
    xkp1(3) = wrap_angle(xkp1(3));
    X_swing(:,k+1) = xkp1;
    U_swing(k+1) = uk;
end

%% --- Sliding Mode Controller (SMC) Simulation (semi-implicit Euler) ---
lambda = 5;      % Sliding surface slope (positive)
k_smc  = 90;     % Switching gain (must be > uncertainty/disturbance)
umax_smc = 50;   % SMC force saturation

x0_SMC = [0; 0; pi-0.5; 0];
X_smc = zeros(4, N); X_smc(:,1) = x0_SMC; U_smc = zeros(1,N);
for k = 1:N-1
    xk = X_smc(:,k);
    uk = smc_controller(xk, m_c, m_p, l, g, b_c, b_p, lambda, k_smc, umax_smc);
    dxk = cartpole_f(xk, uk);
    v_next = xk([2 4]) + dt * dxk([2 4]);
    x_next = xk([1 3]) + dt * v_next;
    xkp1 = [x_next(1); v_next(1); x_next(2); v_next(2)];
    xkp1(3) = wrap_angle(xkp1(3));
    X_smc(:,k+1) = xkp1;
    U_smc(k+1) = uk;
end

%% --- Plotting ---
t = (0:N-1)*dt;
theta_unc = unwrap(X_unc(3,:)');
theta_lqr = unwrap(X_lqr(3,:)');
theta_swing = unwrap(X_swing(3,:)');
theta_smc = unwrap(X_smc(3,:)');

figure;
subplot(4,1,1);
plot(t, X_unc(1,:), 'b', t, theta_unc, 'r');
xlabel('Time (s)'); ylabel('State');
title('Uncontrolled Cart-Pole (θ=π upright)');
legend('Cart Position (m)', 'Pole Angle (rad)');

subplot(4,1,2);
plot(t, X_lqr(1,:), 'b', t, theta_lqr, 'r');
xlabel('Time (s)'); ylabel('State');
title('LQR Controlled Cart-Pole (θ=π upright)');
legend('Cart Position (m)', 'Pole Angle (rad)');

subplot(4,1,3);
plot(t, X_swing(1,:), 'b', t, theta_swing, 'r');
xlabel('Time (s)'); ylabel('State');
title('Swing-Up Controller (θ=π upright)');
legend('Cart Position (m)', 'Pole Angle (rad)');

subplot(4,1,4);
plot(t, X_smc(1,:), 'b', t, theta_smc, 'r');
xlabel('Time (s)'); ylabel('State');
title('Sliding Mode Control (θ=π upright)');
legend('Cart Position (m)', 'Pole Angle (rad)');

figure;
subplot(2,2,1);
plot(t, U_unc, 'b');
xlabel('Time (s)'); ylabel('Force (N)');
title('Uncontrolled');

subplot(2,2,2);
plot(t, U_lqr, 'b');
xlabel('Time (s)'); ylabel('Force (N)');
title('LQR');

subplot(2,2,3);
plot(t, U_swing, 'b');
xlabel('Time (s)'); ylabel('Force (N)');
title('Swing-Up');

subplot(2,2,4);
plot(t, U_smc, 'b');
xlabel('Time (s)'); ylabel('Force (N)');
title('Sliding Mode Control (SMC)');

% --- Energy-based swing-up controller for cart-pole (van den Berg et al., ECC 2007) ---
function u = swingup_controller(x, m_p, l, g, umax, k_swing)
    E_des = m_p * g * l;  % Desired energy (upright)
    theta = x(3);
    dtheta = x(4);
    KE_p = 0.5 * m_p * (l * dtheta)^2;
    PE_p = -m_p * g * l * cos(theta);
    E = KE_p + PE_p;
    dE = E_des - E;
    S = sign(dtheta * cos(theta));  % Direction term
    S(S == 0) = 1;  % Avoid zero
    u = -k_swing * dE * S;  % Control law
    u = max(min(u, umax), -umax);  % Saturate force
end

% --- Sliding Mode Controller for cart-pole upright stabilization ---
function u = smc_controller(x, m_c, m_p, l, g, b_c, b_p, lambda, k, umax)
    x_cart = x(1);
    dx_cart = x(2);
    theta = x(3);
    dtheta = x(4);

    s = dtheta + lambda*wrap_angle(theta - pi);

    % Equivalent control with friction compensation
    u_eq = m_p * g * l * wrap_angle(theta - pi) / (m_c + m_p) ...
         + b_c * dx_cart + b_p * dtheta * l / (m_c + m_p);

    % SMC law with boundary layer
    eps_smc = 0.1;
    sat_s = max(min(s/eps_smc,1),-1);
    u = u_eq - k * sat_s;

    % Saturate force
    u = max(min(u, umax), -umax);
end

function th = wrap_angle(theta)
    th = mod(theta+pi, 2*pi) - pi;
end