
function theta_only
%---------------------initial parameter------------------%
h = 0.01; 
t = 0;
y1 = 10*pi/180;  %theta
y2 = 0;          %theta_dot
x2=0;            
x1=0;
%--------------------------------------------------------%
%Target value: The goal of control
%Current value: Current conditions sensed by the sensor
%Error: The difference between the target value and the error value
%------------------------------------------------------%
Tv=0; %final goal of control
%------------------------------------------------------%
dt=0.002;
kp=-47.3;
kd=-0.8;
C_err_prv=10*pi/180;   %previous error value
F=0;
%----------------------background----------------------------%
x_plane=[-3000 -3000 3000 3000];   
y_plane=[2000 0 0 2000];
x_cart=[-500 500 500 -500];        %---cart
y_cart=[1 1 100 100];
x_pole=[0 0];                      %---pole
y_pole=[100 1000];
R=1000;
plane=fill(x_plane,y_plane,'w');
grid on
hold on
cart=fill(x_cart,y_cart,'B','EraseMode','normal');
pole=line(x_pole,y_pole,'LineWidth',3,'color','k');

%--------------------------------------------------------%
grid on
hold on
%---------Make AVI-----------
%aviobj = avifile('test3.avi')
%----------------------------

pause(3)
for i=1:500   % for 50s
hold on;
grid on;
%---------------------'theta' rungekutta------------------%
k1_1 = f_1(t,y1,y2);
k1_2 = f_2(t,y1,y2,F);
k2_1 = f_1(t+h/2,y1+(k1_1/2)*h,y2+(k1_2/2)*h);
k2_2 = f_2(t+h/2,y1+(k1_1/2)*h,y2+(k1_2/2)*h,F);
k3_1 = f_1(t+h/2,y1+(k2_1/2)*h,y2+(k2_2/2)*h);
k3_2 = f_2(t+h/2,y1+(k2_1/2)*h,y2+(k2_2/2)*h,F);
k4_1 = f_1(t+h,y1+k3_1*h,y2+k3_2*h);
k4_2 = f_2(t+h,y1+k3_1*h,y2+k3_2*h,F);
    
y1 = y1+(t+k1_1+k2_1*2+k3_1*2+k4_1)*h/6;
y2 = y2+(t+k1_2+k2_2*2+k3_2*2+k4_2)*h/6;
%---------------------------------------------------------%

%---------------------'X' rungekutta----------------------%
k11 = f1(t,y1,x2);
k12 = f2(t,y1,x2,F);
k21 = f1(t+h/2,x1+(k11/2)*h,x2+(k12/2)*h);
k22 = f2(t+h/2,x1+(k11/2)*h,x2+(k12/2)*h,F);
k31 = f1(t+h/2,x1+(k21/2)*h,x2+(k22/2)*h);
k32 = f2(t+h/2,x1+(k21/2)*h,x2+(k22/2)*h,F);
k41 = f1(t+h,x1+k31*h,x2+k32*h);
k42 = f2(t+h,x1+k31*h,x2+k32*h,F);
    
x1 = x1+(t+k11+k21*2+k31*2+k41)*h/6;
x2 = x2+(t+k12+k22*2+k32*2+k42)*h/6;
%----------------------------------------------------------%

%------------------PD control------------------------------%
C_err = Tv-y1;   %The difference between the target value and current error
D_err = (C_err-C_err_prv)/dt;
C_err_prv=C_err; 
F=(kp*C_err)+(kd*D_err);
%----------------------------------------------------------%
     x=R*sin(-y1);
     y=R*cos(-y1);
     x_pole=[0 x];
     y_pole=[100 y];
      set(pole,'XData',x_pole,'YData',y_pole)
      updatedX_cart = x_cart + 5000*x1;
      updatedX_pole = x_pole + 5000*x1;
      set(cart,'Xdata',updatedX_cart);
      set(pole,'Xdata',updatedX_pole);
  %-------Make AVI----------------------
      %mo=getframe;
      %aviobj = addframe(aviobj, mo); 
  %-------------------------------------
   drawnow
      t = t + h;
         pause(0.01)
%---------state, input force--------------------   
figure(2);

     subplot(511)
     axis([0 2.5 -10 10])
     plot(t,F,'.','MarkerSize',15,'color','y')
     hold on
     grid on
     ylabel('Input Force')
     
     subplot(512)
     axis([0 2.5 -0.2 0.2])
     plot(t,y1,'.','MarkerSize',15,'color','b')
     grid on
     hold on
     ylabel('theta(rad)')
      
     subplot(513)
     axis([0 2.5 -2 2])
     plot(t, y2,'.','MarkerSize',15,'color','b')
     grid on;
     hold on;
     ylabel('theta(dot)')
     
     subplot(514)
     axis([0 2.5 -0.5 0.5])
     plot(t,x1,'.','MarkerSize',15,'color','r')
     grid on
     hold on
     ylabel('X')
     
     subplot(515)
     axis([0 2.5 -2 2])
     plot(t, x2,'.','MarkerSize',15,'color','r')
     grid on;
     hold on;
     ylabel('X(dot)')
end
%--------Make AVI------
%aviobj=close(aviobj);
%----------------------

function y2dot = f_2(t,y1,y2,F)

y2dot = 16.386*y1-0.986*F; 

function y1dot = f_1(t,y1,y2)
y1dot = y2;

function x2dot = f2(t,y1,x2,F)
x2dot = 0.57836*y1-0.62295*F;

function x1dot = f1(t,y1_0,x2)
x1dot = x2;
%%%%%%
