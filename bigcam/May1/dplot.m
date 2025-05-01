clear
close all

CPI = 16216; % encoder counts per inch

load '5.txt'
A=X5;

ticks = A(:,1)-A(1,1);
h=0.01;   % 1s sampling interval
t=ticks*h;

perr = A(:,5);
u    = A(:,6);
ui   = A(:,7);
cmd   = A(:,8);
hold  = A(:,15);
kp    = A(:,12);
ki    = A(:,13);

figure('Name','perr')
plot(t,perr/CPI,'r')
xlabel('time (s)')
ylabel('perr')

figure('Name','u & ui')
plot(t,u,'k', t,ui,'b', t,cmd,'m', t,perr/CPI,'r', t,hold,'k:')
xlabel('time (s)')
legend('u(v)','ui(v)','cmd','err(inches)','hold')

figure('Name','cmd')
plot(t,cmd,'g')