% plant simulation
clear
close all

load cmd.txt
load lvdt.txt
load edge.txt

fs = 100;
N = length(lvdt);
t=0:1/fs:(N-1)/fs;

figure('Name','Dout')
plot(t,cmd,'ro-',t,lvdt,'bo-',t,edge,'go-')