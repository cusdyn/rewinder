clear
close all

% Dayton Motor into 2500 count per rev encoder

% Identified model
K= 20000;
Tm=2;

nump = K;
denp = conv([1 0],[Tm 1])

figure('Name','Plant')
bode(nump,denp)
grid on

% assume an integrator. kill the pole at w=0.1
Kp=1e-5;
numc = Kp*[1 .1];
denc = [1 0];

figure('Name','System')
num = conv(nump, numc);
den = conv(denp, denc);
bode(num,den)
grid on

figure('Name','Root Locus')
rlocus(num,den)
grid on

H = tf(num,den)
h = 0.01;        % 100Hz sampling

Hd=c2d(H,h,'zoh')

step(feedback(Hd,1))
