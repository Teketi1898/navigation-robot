# navigation-robot
The code implements a basic potential field-based path planning algorithm for a robot navigating through an environment with obstacles to reach a specified goal
Code for creating Environment with Robot, Obstacles & goal 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Position of obstacles and goals 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Obstacle 1
K1=0.5; 
Obs1=[3.5 1.5]; 
%Obstacle 2
K2=0.5; 
Obs2=[3 3.4]; 
%Obstacle 3
K3=0.5; 
Obs3=[4 6]; 
%Obstacle 4
K4=0.5; 
Obs4=[8 6.7]; 
%Obstacle 5
K5=0.5; 
Obs5=[5.8 4]; 
%Obstacle 6
K6=0.5; 
Obs6=[1.4 8]; 
%obstacle7
K7=0.5; 
Obs7=[9 9]; 
%Goal
Kg=0.1; 
Goal=[9 6]; 
%Robot
Kr=2; 
Robot=[0.5 0.5]; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Original Robot Environment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
circleBlack(Robot(1)*10,Robot(2)*10,2.9,4); %robot 1
circleGreen(Goal(1)*10,Goal(2)*10,7,8);%goal right
square_red(Obs1(1)*10,Obs1(2)*10,3.5,6);%obstacle
square_red(Obs2(1)*10,Obs2(2)*10,3.4,6);%obstacle
square_red(Obs3(1)*10,Obs3(2)*10,4.6,6);%obstacle
square_red(Obs4(1)*10,Obs4(2)*10,3.9,6);%obstacle
square_red(Obs5(1)*10,Obs5(2)*10,5,6);%obstacle
square_red(Obs6(1)*10,Obs6(2)*10,5.9,6);%obstacle
square_red(Obs7(1)*10,Obs7(2)*10,4.5,6);%obstacle
figure (1) 
title('Environment') 
hold on
axis([0 100 0 100]); 
grid on; 
Repulsive and attractive force to avoid obstacles. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create Repulsive and attractive potential field forces

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Matrix X x Y of 100by100
i=1:100; 
j=1:100; 
[I,J] = meshgrid(i,j); 
Z = []; %empty matrix for z-axis plot
for i=1:100 
 for j=1:100 
 pos = [j/10 i/10]; % scaled to fit 100by100 matrix
 z=0; %accumulator
 
 %Obstacles with exponential curve while goal is by vector 
 z=z+exp(-norm(pos-Obs1)/K1)+ ...
 exp(-norm(pos-Obs2)/K2)+ ...
 exp(-norm(pos-Obs3)/K3)+ ...
 exp(-norm(pos-Obs4)/K4)+ ...
 exp(-norm(pos-Obs5)/K5)+ ...
 exp(-norm(pos-Obs6)/K6)+ ...
 exp(-norm(pos-Obs7)/K7)+ ...
 Kg*(norm(Goal-pos)); 
 % The potential functions chosen are similar to the function proposed in
 % the document 'apf.pdf'
 Z(i,j) = z; %store in matrix
 
 end
end
 t=(abs(Robot(1)-Goal(1))+abs(Robot(2)-Goal(2))); 
count=0; 
while t>0.8 %check if the robot had reached the specified target; stop when reach 
target
 % previously t>0.1
 dx=[0 0]; %accumulation matrix
 
 %Obstacles Accumulate vector sum %Goal Accumulate
 dx= dx+(Robot-Obs1)*exp((-norm(Robot-Obs1))/K1)+ ...
 (Robot-Obs2)*exp((-norm(Robot-Obs2))/K2)+ ...
 (Robot-Obs3)*exp((-norm(Robot-Obs3))/K3)+ ...
 (Robot-Obs4)*exp((-norm(Robot-Obs4))/K4)+ ...
 (Robot-Obs5)*exp((-norm(Robot-Obs5))/K5)+ ...
 (Robot-Obs6)*exp((-norm(Robot-Obs6))/K6)+ ...
 (Robot-Obs7)*exp((-norm(Robot-Obs7))/K7)+ ...
 Kg*(Goal-Robot)/norm(Goal-Robot); 
 
 %Change in Robot Position over time (differenttiation)
 Robot=Robot+dx; 
 
 %Calculate the value difference between the robot and the goal 
 t=(abs(Robot(1)-Goal(1))+abs(Robot(2)-Goal(2))); 
 count=count+1; 
 %update display 
 %calculate distance between robot and goal
 plot(Robot(1,1)*10, Robot(1,2)*10, 'K.','MarkerSize',20); 
 title(sprintf('Iteration: %d',count)); 
 refresh; 
 drawnow; 
end 


Plotting of the navigation of the path from the robot position to the goal. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot Figures
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2) %Contour figure
title('Contour') 
hold on
contour(Z,22) %plot figure with 22 contours 
pause(2) 
hold on; 
figure(3) %combine figures
title('Potential Force Field on Contour') 
hold on
contour(Z,22) %plot figure with 22 contours 
hold on
[px,py]=gradient(Z); %calculate gradient 
quiver(I,J,-px,-py,'r'), hold on % plot velocity vectors
pause(3) 
figure(4) 
title('Potential Function Landscape') 
hold on; grid on
surfc(Z) %plot surface and contour plot
view([35,100,30]) 
pause(1) 
hold on;
