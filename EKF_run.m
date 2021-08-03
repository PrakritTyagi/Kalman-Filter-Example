clear
close all
clc
%% Prakrit Tyagi : EKF Program tracking a vehicle with acceleration and range measurements
% date : 3 august 2021

% constants
del_t = 1; % discrete time 1 sec
total_time = 300; % Total simulation time in seconds

% Beacon parameters
h = 500;
x1 = -1000;
x2 = 3000;
x3 = 4000;

% To save simulated and estimated state values
X_est = zeros(2,total_time+1);
X_true = zeros(2,total_time+1);

% initialize
x_true = [0;0]; % initial true state 
x_guess = [2;2]; % guess for initialization of states
X_est(:,1) = x_guess;
X_true(:,1)= x_true;

P = [2,0;0,2]; % guess of state estimation covariance matrix 
% Q = [0.01 0 ;0 0.1];
sd_a = 0.5; % Assuming standard deviation of acceleration for process noise covariance matrix
Q = sd_a^2*[0.25*del_t^4, 0.5*del_t^3 ; 0.5*del_t^3, del_t^2]; % Process noise covariance matrix

%% Kalman Filter simulation
fig1 = figure;
line = 1:1:(total_time+1);

for i = 0:1:(total_time-1)
    
    b = 0.01; % bias in acceleration measurement
    a_noise = normrnd(0,0.001); % random noise in acceleration
    if i<100
        a_true = (0.05/100)*i;
        a_meas = a_true + b + a_noise;
    elseif i<=100 && i <200
        a_true = 0.05;
        a_meas = a_true + b + a_noise;
    else 
        a_true = -(0.05/100)*i + 3*0.05;
        a_meas = a_true + b + a_noise;
    end
    
    F = [1, del_t;0, 1]; % F matrix for state prediction equation
    G = [0.5*del_t^2 ; del_t]; % G matrix for state prediction equation
    
    % Prediction Equation for state and P
    X_est(:,i+2)= F*X_est(:,i+1) + G*a_meas;
    P = F*P*F' + Q;
    
    % True Next state of vehicle
    a = a_true + normrnd(0,0.5);
    X_true(:,i+2)= F*X_true(:,i+1) + G*a;
    
    
    %simulate range measurement 
     r1 = sqrt( (X_true(1,i+2) - x1)^2 + h^2 ) + normrnd(0,1);
     r2 = sqrt( (X_true(1,i+2) - x2)^2 + h^2 ) + normrnd(0,1);
     r3 = sqrt( (X_true(1,i+2) - x3)^2 + h^2 ) + normrnd(0,1);
     z = [r1;r2;r3]; % range measurement vector
     
     r1_est = sqrt( (X_est(1,i+2) - x1)^2 + h^2 ) ;
     r2_est = sqrt( (X_est(1,i+2) - x2)^2 + h^2 ) ;
     r3_est = sqrt( (X_est(1,i+2) - x3)^2 + h^2 ) ;
     z_est = [r1_est;r2_est;r3_est]; % estimating range measurements
     
     % Jacobian matrix 
     H = [ (X_est(1,i+2) - x1)/r1_est, 0; (X_est(1,i+2) - x2)/r2_est, 0; (X_est(1,i+2) - x3)/r3_est, 0];
     
     % Measurement covariance matrix
     R = [1.5 0 0; 0 1.5 0; 0 0 1.5];
     
     % Kalman Gain
     K = P*(H')/(H*P*H' + R);
     
     % Update Equations for state and P (Correction of prediction)
     X_est(:,i+2) = X_est(:,i+2) + K*(z - z_est);
     P = (eye(2) - K*H)*P*(eye(2) - K*H)' + K*R*K';
     
     
    
    clf 
    hold on
    plot( line(1,1:(i+1)), X_est(1,1:(i+1)),line(1,1:(i+1)), X_true(1,1:(i+1)),'--')
    grid on
    xlabel('time') 
    ylabel('xtrue vs xest') 
    drawnow
    
    i
 
end

fig2 = figure;
plot(line, X_est(2,:),line, X_true(2,:),'--')
grid on
xlabel('time') 
ylabel('v true vs v est')

fig3 = figure;
plot(line, X_est(1,:)- X_true(1,:))
grid on
xlabel('time') 
ylabel('Error in position')

fig4 = figure;
plot(line, X_est(2,:)- X_true(2,:))
grid on
xlabel('time') 
ylabel('Error in velocity')