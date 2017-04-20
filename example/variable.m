% Illustrates how it is possible to get practically the same transfer
% by varying frequency and order of the transfer function. 
B = tf([1/(25*2*pi) 1],[1/(20*2*pi) 1]); B2 = tf([1/(27.7*2*pi) 1],[1/(17.7*2*pi) 1]);
T = B^2; figure; hold on; bode(T,P); bode(B,P); bode(B2,P);
legend('2nd order function','1st order function','1st order function');

%% 
% This demonstrates the use of FOC. If a higher/lower integer order is not possible
% because of slope /  roll-off issues, FOC is legit. Otherwise playing with
% integer orders and frequencies might be an easier solution. 