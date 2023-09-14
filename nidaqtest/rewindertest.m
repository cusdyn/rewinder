clear
close all
load  edge.txt
load  rack.txt
load  frack.txt
load  drack.txt
load  edgeerr.txt
load  speederr.txt
load  tdout.txt
load cmdout.txt

h=1;  % sampling interval (seconds)
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


figure('Name',"Errors")
plot(t,edgeerr,'r',t,speederr,'b',t,cmdout-5,'m')
legend('edge err','speed err','cmd')

figure('Name',"noise check")
plot(t,rack,'r',t,edge,'b')
legend('rack','edge')