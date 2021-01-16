%% plot 3D

for n=1:300
  
  l=48;w=38;h=6;
  x=out.Xe.Data(n,1);y=out.Xe.Data(n,2);z=out.Xe.Data(n,3); 
  roll=out.rotation.Data(n,1);pitch=out.rotation.Data(n,2);yaw=out.rotation.Data(n,3);
   
  hw_x=[x+(l/2)*cos(roll)*cos(pitch),x-(l/2)*cos(roll)*cos(pitch)];
  hw_y=[y+(l/2)*sin(roll)*cos(pitch),y-(l/2)*sin(roll)*cos(pitch)];
  hw_z=[z-(l/2)*sin(pitch),z+(l/2)*sin(pitch)];
  zy_x=[x+(w/2)*(cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw)),x-(w/2)*(cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw))];
  zy_y=[y+(w/2)*(sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw)),y-(w/2)*(sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw))];
  zy_z=[z+(w/2)*(cos(pitch)*sin(yaw)),z-(w/2)*(cos(pitch)*sin(yaw))];
  sx_x=[x+(h/2)*(cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw)),x-(h/2)*(cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw))];
  sx_y=[y+(h/2)*(sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw)),y-(h/2)*(sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw))];
  sx_z=[z+(h/2)*(cos(pitch)*cos(yaw)),z-(h/2)*(cos(pitch)*cos(yaw))];
  
  plot3(hw_x,hw_y,hw_z)
  hold on
  plot3(zy_x,zy_y,zy_z)
  hold on
  plot3(sx_x,sx_y,sx_z)
  xlabel('x');ylabel('y');zlabel('z');
  grid on;
  hold on;
  
  l=48;w=38;h=6;
  x1=out.Xe1.Data(n,1);y1=out.Xe1.Data(n,2);z1=out.Xe1.Data(n,3);
  roll1=out.rotation1.Data(n,1);pitch1=out.rotation1.Data(n,2);yaw1=out.rotation1.Data(n,3);
  
  hw_x1=[x1+(l/2)*cos(roll1)*cos(pitch1),x1-(l/2)*cos(roll1)*cos(pitch1)];
  hw_y1=[y1+(l/2)*sin(roll1)*cos(pitch1),y1-(l/2)*sin(roll1)*cos(pitch1)];
  hw_z1=[z1-(l/2)*sin(pitch1),z1+(l/2)*sin(pitch1)];
  zy_x1=[x1+(w/2)*(cos(roll1)*sin(pitch1)*sin(yaw1)-sin(roll1)*cos(yaw1)),x1-(w/2)*(cos(roll1)*sin(pitch1)*sin(yaw1)-sin(roll1)*cos(yaw1))];
  zy_y1=[y1+(w/2)*(sin(roll1)*sin(pitch1)*sin(yaw1)+cos(roll1)*cos(yaw1)),y1-(w/2)*(sin(roll1)*sin(pitch1)*sin(yaw1)+cos(roll1)*cos(yaw1))];
  zy_z1=[z1+(w/2)*(cos(pitch1)*sin(yaw1)),z1-(w/2)*(cos(pitch1)*sin(yaw1))];
  sx_x1=[x1+(h/2)*(cos(roll1)*sin(pitch1)*cos(yaw1)+sin(roll1)*sin(yaw1)),x1-(h/2)*(cos(roll1)*sin(pitch1)*cos(yaw1)+sin(roll1)*sin(yaw1))];
  sx_y1=[y1+(h/2)*(sin(roll1)*sin(pitch1)*cos(yaw1)-cos(roll1)*sin(yaw1)),y1-(h/2)*(sin(roll1)*sin(pitch1)*cos(yaw1)-cos(roll1)*sin(yaw1))];
  sx_z1=[z1+(h/2)*(cos(pitch1)*cos(yaw1)),z1-(h/2)*(cos(pitch1)*cos(yaw1))];
  
  plot3(hw_x1,hw_y1,hw_z1)
  hold on
  plot3(zy_x1,zy_y1,zy_z1)
  hold on
  plot3(sx_x1,sx_y1,sx_z1)
  hold on
  xlabel('x');ylabel('y');zlabel('z');  
  %axis([-50,10050,-35,150,7300,8020]);
  drawnow
  frame(n) = getframe(gcf);
  pause(0.005);
  hold off
end

camlight




% ein Video herstellen
v = VideoWriter('plane.avi');  % Das erzeugte Video heisst 'trackOFplane.avi'.
v.FrameRate = 5;     % Die Abspielengeschwindigkeit des Videos (wie viele Bilder pro Sekunkde)
v.Quality = 90;            % Videoqualitaet laesst zwischen 0 und 100 eingestellt. Standardmassig ist 75.
open(v);
 for i = 1:1:300
     writeVideo(v,frame(i));
 end
    close(v)


