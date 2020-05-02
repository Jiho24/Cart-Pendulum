
function theta_x
%---------------------initial parameter------------------%
h = 0.005;
t = 0;
y1 = 10*pi/180;  %theta
y2 = 0;         %theta dot
x2=0;
x1=0;
%--------------------------------------------------------%
dt=0.000155;
dtx=0.000155;
%---------------------------
Tv=0;          %The goal of control 
%---------------------------
kp=-150;
kd=-2;
kpx=1;         
kdx=2.35;
%---------------------------
C_err_prv=0;
Cx_err_prv=0;
F=0;
F_x=0;
T_F=0; %Total Force
%---------------------------
x_plane=[-3000 -3000 3000 3000];   %---background
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
%--------------------------------------------
%---------Make AVI-----------
%aviobj = avifile('test6.avi')
%----------------------------
pause(3)
for i=1:1000   % for 50s
hold on;
grid on;

k1_1 = f_1(t,y1,y2);
k1_2 = f_2(t,y1,y2,T_F);
k2_1 = f_1(t+h/2,y1+(k1_1/2)*h,y2+(k1_2/2)*h);
k2_2 = f_2(t+h/2,y1+(k1_1/2)*h,y2+(k1_2/2)*h,T_F);
k3_1 = f_1(t+h/2,y1+(k2_1/2)*h,y2+(k2_2/2)*h);
k3_2 = f_2(t+h/2,y1+(k2_1/2)*h,y2+(k2_2/2)*h,T_F);
k4_1 = f_1(t+h,y1+k3_1*h,y2+k3_2*h);
k4_2 = f_2(t+h,y1+k3_1*h,y2+k3_2*h,T_F);
    
y1 = y1+(k1_1+k2_1*2+k3_1*2+k4_1)*h/6;
y2 = y2+(k1_2+k2_2*2+k3_2*2+k4_2)*h/6;
%--------------------------------------------------------%
k11 = f1(t,y1,x2);
k12 = f2(t,y1,x2,T_F);
k21 = f1(t+h/2,x1+(k11/2)*h,x2+(k12/2)*h);
k22 = f2(t+h/2,x1+(k11/2)*h,x2+(k12/2)*h,T_F);
k31 = f1(t+h/2,x1+(k21/2)*h,x2+(k22/2)*h);
k32 = f2(t+h/2,x1+(k21/2)*h,x2+(k22/2)*h,T_F);
k41 = f1(t+h,x1+k31*h,x2+k32*h);
k42 = f2(t+h,x1+k31*h,x2+k32*h,T_F);
    
x1 = x1+(k11+k21*2+k31*2+k41)*h/6;
x2 = x2+(k12+k22*2+k32*2+k42)*h/6;
%--------------------------------------------------------%
C_err = Tv-y1;
D_err = (C_err-C_err_prv)/dt;
C_err_prv=C_err;
F=(kp*C_err)+(kd*D_err);
%--------------------------------------------------------%
Cx_err = Tv-x1;
Dx_err = (Cx_err-Cx_err_prv)/dtx;
Cx_err_prv=Cx_err;
F_x=(kpx*Cx_err)+(kdx*Dx_err);
%--------------------------------------------------------%
T_F=(F_x+F);
%--------------------------------------------------------%
t = t + h;
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
%       mo=getframe;
%       aviobj = addframe(aviobj, mo); 
  %-------------------------------------
     drawnow
     pause(0.001)
     figure(2);

     subplot(511)
     axis([0 2.5 -100 100])
     plot(t,F,'.','MarkerSize',10,'color','y')
     hold on
     grid on
     ylabel('Input Force')
     
     subplot(512)
     axis([0 2.5 -1 1])
     plot(t,y1,'.','MarkerSize',10,'color','b')
     grid on
     hold on
     ylabel('theta(rad)')
      
     subplot(513)
     axis([0 2.5 -6 6])
     plot(t, y2,'.','MarkerSize',10,'color','b')
     grid on;
     hold on;
     ylabel('theta(dot)')
     
     subplot(514)
     axis([0 2.5 -0.7 0.7])
     plot(t,x1,'.','MarkerSize',10,'color','r')
     grid on
     hold on
     ylabel('X')
     
     subplot(515)
     axis([0 2.5 -6 6])
     plot(t, x2,'.','MarkerSize',10,'color','r')
     grid on;
     hold on;
     ylabel('X(dot)')
end
%--------Make AVI------
%aviobj=close(aviobj);

%-----------state, input force----------------
        
    

function y2dot = f_2(t,y1,y2,TF)

y2dot = 16.386*y1-0.986*TF; 

function y1dot = f_1(t,y1,y2)
y1dot = y2;

function x2dot = f2(t,y1,x2,TF)
x2dot = 0.57836*y1-0.62295*TF;

function x1dot = f1(t,y1,x2)
x1dot = x2;

