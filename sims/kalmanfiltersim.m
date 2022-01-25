%Gyro test program 

%start with clear memory and close all plots 

clear all;
close all;

%gyro and accelerometer sim params

rollTruek1 = 0.0;
rollTruek = 0.0;
rollVelTrue = 0.0;
gyroDriftTrue = 3.0;
gyroBiasTrue = 0.01;
gyroSigmaNoise = 0.1;
accelSigmaNoise = sqrt(0.5);

accelMeask1 = 0.0;
accelMeask = 0.0;
gyroMeask1 = 0.0;
gyroMeask = 0.0;

deltaT = 0.004;%250Hz
time = 0.0;
dataStore = zeros(5010,7);

%complemetary filter parameters 
angleRollk1 = 0.0;
angleRollk = 0.0;

gaink1 = 0.90;%gyro gain 
gaink2 = 0.10;%drift correction gain 

%kalman filter params

xk = zeros(1,2);     %state vector 1 angle 2 gyro drift
pk = [0.5 0 0 0.01]; % covarience matrix 
K = zeros(1,2);      %Kalman gain 
phi = [1 deltaT 0 1];%state transition matrix
psi = [deltaT 0];    %control transition matrix 
R = 0.03;            %measurement noise
Q = [0.002^2 0 0 0]; %proccess noise
H = [1 0];           %observation matrix

%sim loop

endPoint = 2500;
for i = 1:endPoint;
    % moving roll of gyro 
    rollVelTrue = 0.0;
    if (i>=500) && (i<750);
        rollVelTrue = 10.0;
    end
    if (i>=1250) && (i<1500);
        rollVelTrue = -15.0;
    end
    if (i>=1750) && (i<2000);
        rollVelTrue = 5.0;
    end
    

    %create gyro and accel measurements 
    % gyro in [deg/sec] accel with change g to angle [deg] 
    
    rollTruek1 = rollTruek + rollVelTrue*deltaT; %integrate for val of gyro
    
    accelMeask1 = rollTruek1 + randn(1)*accelSigmaNoise; % in degrees 
    
    gyroReadk1 = rollVelTrue + gyroDriftTrue + randn(1)*...
        gyroSigmaNoise + gyroBiasTrue;
    
    gyroMeask1 = gyroReadk1 - gyroBiasTrue;
    
    
    %complemetary filter 
    
    angleRollk1 = gaink1*(angleRollk + gyroMeask1*deltaT)...
         + gaink2*(accelMeask1);
    
    %kalman filter
    %estimating roll and gyro bias in scaler form
    
    uk = gyroMeask1;
    zk = accelMeask1;
    
    %xmk1Minus = phi*xk + psi*uk;
    xk1Minus(1) = phi(1)*xk(1) + phi(2)*xk(2) + psi(1)*uk;
    xk1Minus(2) = phi(3)*xk(1) + phi(4)*xk(2) + psi(2)*uk;
    
    %pk1Minus = phi*pk*phi' + Q;
    pk1Minus(1) = (phi(1)*pk(1) + phi(2)*pk(3))*phi(1) +...
                  (phi(1)*pk(2) + phi(2)*pk(4))*phi(2) + Q(1);
    pk1Minus(2) = (phi(1)*pk(1) + phi(2)*pk(3))*phi(3) +...
                  (phi(1)*pk(2) + phi(2)*pk(4))*phi(4) + Q(2);
    pk1Minus(3) = (phi(3)*pk(1) + phi(4)*pk(3))*phi(1) +...
                  (phi(3)*pk(2) + phi(4)*pk(4))*phi(2) + Q(3);    
    pk1Minus(4) = (phi(3)*pk(1) + phi(4)*pk(3))*phi(3) +...
                  (phi(3)*pk(2) + phi(4)*pk(4))*phi(4) + Q(4); 
    
    %S = H*pk1Minus*H' + R;
    S = (H(1)*pk1Minus(1) + H(2)*pk1Minus(3))*H(1) +...
        (H(1)*pk1Minus(2) + H(2)*pk1Minus(4))*H(1) + R;
    
    %K = pk1Minus*H' + R;
    K(1) = (pk1Minus(1)*H(1) + pk1Minus(2)*H(2))/S;
    K(2) = (pk1Minus(3)*H(1) + pk1Minus(4)*H(2))/S;
    
    %xk1 = ck1Minus + K*(zk - H*xk1Minus);
    xk1(1) = xk1Minus(1) + K(1)*(zk - (H(1)*xk1Minus(1) +...
             H(2)*xk1Minus(2)));
    xk1(2) = xk1Minus(2) + K(2)*(zk - (H(1)*xk1Minus(1) +...
             H(2)*xk1Minus(2)));
    
    %pk1 = (eye(2,2) - K*H)*pk1Minus;
    pk1(1,1) = (1 - K(1)*H(1))*pk1Minus(1) +...
               (0 - K(1)*H(2))*pk1Minus(3);
    pk1(1,2) = (1 - K(1)*H(1))*pk1Minus(2) +...
               (0 - K(1)*H(2))*pk1Minus(4);
    pk1(2,1) = (1 - K(2)*H(1))*pk1Minus(1) +...
               (0 - K(2)*H(2))*pk1Minus(3);
    pk1(2,2) = (1 - K(2)*H(1))*pk1Minus(2) +...
               (0 - K(2)*H(2))*pk1Minus(4);
    
    
    %store data 
    dataStore(i,:) = [time rollTruek1 accelMeask1 gyroMeask1 ...
        angleRollk1 xk1(1) xk1(2)];
    %reset values for next iteration
    angleRollk = angleRollk1;
    rollTruek = rollTruek1;
    time = time + deltaT;
    
    xk = xk1;
    pk = pk1;
end

%plot results

plot(dataStore(1:endPoint,1),dataStore(1:endPoint,2),'r');
hold
plot(dataStore(1:endPoint,1),dataStore(1:endPoint,5),'g');
plot(dataStore(1:endPoint,1),dataStore(1:endPoint,6),'b');
grid on
axis([0 10 -20 20])
xlabel('time[sec]')
ylabel('angle[deg]')
legend('True angle','complementary filter','kalman filter')










