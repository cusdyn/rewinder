clear
close all
load  edge.txt
load  rack.txt
load  frack.txt
load  drack.txt
load  edgeerr.txt
load  speederr.txt
load  tdout.txt

h=1/400;  % sampling interval (seconds)
N=length(edge);

t=0:h:h*(N-1);

figure('Name',"Raw and filtered rack output")
plot(t,rack,'r-o',t,frack,'b-');
legend('raw','filtered')

figure('Name',"drack")
plot(t,drack,'r-o');

figure('Name',"Edge Guide")
plot(t,edge,'b-o');

figure('Name',"timediff")
plot(t(2:N),tdout(2:N),'b-o')