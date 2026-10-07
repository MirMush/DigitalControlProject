dd = load('basebot_6_50/logs/1ms-sampletime.log');
h = figure(1000);
hold off
plot(dd(:,1), dd(:,4))
hold on
plot(dd(:,1), dd(:,8)/100)
plot(dd(:,1), dd(:,14))  
yline(5.13)
grid on
legend('Motor Voltage left (V)', 'Motor velocity left (rad/s)', 'Motor Current left (A)')
xlabel('Time (sec)')

%%
dIss = iss3 - iss2
dwss = (wss3 - wss2)*50 

%%
% 1. Define the given variables
D = 0.0000188; 
L = 5.15e-3; 
R = 4.29; 
K_b = 1.34e-2; 
K_t = 1.34e-2; 
s = -27.59;

% 2. Calculate coefficients for the Left Hand Side (LHS) before squaring
% LHS format: A*J + B
A = 2 * L * s + R;
B = D * L;

% 3. Calculate the J^2 and J^1 coefficients for LHS after squaring
% (A*J + B)^2 = A^2 * J^2 + 2*A*B * J + B^2
LHS_J2 = A^2;
LHS_J1 = 2 * A * B;
% Note: The constant term B^2 (which is D^2 * L^2) perfectly cancels out 
% with the constant term on the RHS, so we can ignore it.

% 4. Calculate the J^2 and J^1 coefficients for the Right Hand Side (RHS) inside the root
RHS_J2 = R^2;
RHS_J1 = -2 * D * L * R - 4 * K_b * K_t * L;

% 5. Group the terms to form the equation: (RHS_J2 - LHS_J2)*J^2 + (RHS_J1 - LHS_J1)*J = 0
a = RHS_J2 - LHS_J2;
b = RHS_J1 - LHS_J1;

% 6. Solve for the non-zero J (divide by J, leaving a*J + b = 0)
J = -b / a;

% Display the final result
fprintf('The calculated value of J is: %e\n', J);

% ---------------------------------------------------------
% Optional: Verification block
% ---------------------------------------------------------
term_under_sqrt = D^2 * L^2 - 2*D*J*L*R + J^2 * R^2 - 4*J*K_b*K_t*L;

% Calculate s using the PLUS sign branch (as noted in the mathematical breakdown)
s_check = ( -(D*L + J*R) + sqrt(term_undersequenceTwoSteps_sqrt) ) / (2 * J * L);
fprintf('Verification (s target = %g): Calculated s = %g\n', s, s_check);