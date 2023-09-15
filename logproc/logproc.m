clear
%close all

load '143.txt'
A=X143;

ticks = A(:,1)-A(1,1);
h=0.01;   % 1s sampling interval
t=ticks*h;

Vedge = A(:,2);
Lvdt  = A(:,3);

figure('Name','Vedge')
plot(t,Vedge,'ro-',t,Lvdt,'bo')