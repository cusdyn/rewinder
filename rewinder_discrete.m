% Zero-Order Hold equivalent for Identified Model H(S)=K/[s(s+a)]
clear
close all

% System ID result
K = 20;  % Plant Gain: amplifier input to LVDT output
a = 25;  % Plant bandwidth (rad/sec)


numc = K;
denc = [1 a 0];

figure('Name',"Plant Model Frequency Response");
bode(numc,denc);
xlim([0.1 100]);

% zero-order hold equivalent

h = 0.01;  % 10ms sampling time

H = tf(numc,denc);
G = c2d(H,h,'zoh');

[numd,dend,ts]=tfdata(G);
figure('Name',"Continuous and discrete impulse responses");
impulse(H,'-',G,'--')

figure('Name',"root locus");
rlocus(G)

figure('Name',"Plant Model Frequency Response");
dbode(numd,dend,h);

% discrete impulse response
u = zeros(1,100);
y=u;
u(1) = 1;

N=length(u);

b1 = numd{1}(2)
b0 = numd{1}(3)
a1 = dend{1}(2)
a0 = dend{1}(3)

for k=1:N,
    if k>2
        y(k) = b1*u(k-1) + b0*u(k-2) - a1*y(k-1) - a0*y(k-2);
    elseif k==2
        y(k) = b1*u(k-1) - a1*y(k-1);
    else  % k=1
        y(k)=0;
    end
end

figure('Name','Discrete impulse response')
plot(y)