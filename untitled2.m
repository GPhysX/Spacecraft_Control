clc;
clear all;
close all;
 
%-------------------------------------------------------------------------
% measurements
 
x = [1, 5, 10, 15, 20,  1, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80];
y = [1, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80];
 
VxMeas = [5,5,5,5,5,20,5,5,5,5,5,5,5,5,5,5,5];
VyMeas = [5,5,5,5,5,0,5,5,5,5,5,5,5,5,5,5,5];
 
%-------------------------------------------------------------------------
% kalman filter
 
X = [0;  %x
     0;  %y
     0;  %Vx
     0]; %Vy
 
Q = 10;
% Vx = 5;
% Vy = 5;
Ts = 1;
P = eye(4);
 
%state transition matrix
A = [1, 0, Ts, 0;
     0, 1, 0, Ts;
     0, 0, 1, 0;
     0, 0, 0, 1;
    ];
 
%transformation matrix C
C = [1, 0, 0,0;
     0, 1, 0,0];
       
% initialize measurement noise covar matrix R
 R = [1, 0, 0, 0;
      0, 1, 0, 0;
      0, 0, 1, 0;
      0, 0, 0, 1;];
 
% calculate measurement noise covariance matrix R
   myMeanX = mean(x);
   myMeanY = mean(y);
   varianceX = 0;
   varianceY = 0;
   for i=1:length(x)
        varianceX = varianceX + (myMeanX - x(i))^2;
        varianceY = varianceY + (myMeanY - y(i))^2;
   end
   
  myMeanVx = mean(VxMeas);
  myMeanVy = mean(VyMeas);
  varianceVx = 0;
  varianceVy = 0;
  for i=1:length(varianceVx)
       varianceVx = varianceVx + (myMeanVx - VxMeas(i))^2;
       varianceVy = varianceVy + (myMeanVy - VyMeas(i))^2;
  end    
 
    R(1,1) = varianceX/length(x);
    R(2,2) = varianceY/length(y);
    R(3,3) = varianceVx/length(VxMeas);  
    R(4,4) = varianceVy/length(VyMeas);  
   
I = [1, 0, 0, 0;
     0, 1, 0, 0;
     0, 0, 1, 0;
     0, 0, 0, 1];
 
 msqX =0;
 msqY =0;
 
for k=1:length(x)
disp('calculated: ');
R
   
     Y = [x(k);
          y(k);
          VxMeas(k);
          VyMeas(k)];
   
   %prediction
        X = A*X;
        P = A*P*A.'+Q;
 
 
   %correction
        K = P/(P+R);
        X = X + K*(Y-X);
        XPosCorrected(k) = X(1,1);
        YPosCorrected(k) = Y(2,1);
       
        %step 3
        P = (I - K)*P;
        kalmanGain(k) = (K(1,1)+K(2,2))/2;
       
       
end    
 
figure(1);
subplot(2,1,1)
plot(x,y, '-bo');
hold on;
plot(XPosCorrected, YPosCorrected, '-mo');
title('input output')
legend('measurements', 'output Kalman');
subplot(2,1,2)
stem(kalmanGain);
title('kalman gain')