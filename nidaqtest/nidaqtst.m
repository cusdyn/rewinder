clear
close all
load  dout.txt
load  vout.txt
load  tdout.txt
load  egout.txt
load eout.txt

h=1/400;  % sampling interval (seconds)
N=length(dout);

t=0:h:h*(N-1);

dv = zeros(1,N);
dvf = zeros(1,N);



figure('Name',"Raw and filtered output")
%plot(t,dout,'b-o',t,vout,'r-+',t,dv,'g')
%plot(t,dout,'b-o',t,vout,'r-+')
plot(t,dout,'r-o',t,vout,'b-');
legend('raw','filtered')

figure('Name',"Rack Error")
%plot(t,dout,'b-o',t,vout,'r-+',t,dv,'g')
%plot(t,dout,'b-o',t,vout,pwd'r-+')
plot(t,eout/80,'r-');

figure('Name',"Edge Guide")
%plot(t,dout,'b-o',t,vout,'r-+',t,dv,'g')
%plot(t,dout,'b-o',t,vout,pwd'r-+')
plot(t,egout,'b-o');



figure('Name',"timediff")
plot(t(2:N),tdout(2:N),'b-o')