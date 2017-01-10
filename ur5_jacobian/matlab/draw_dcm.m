function draw_dcm(dcm)
dcm = dcm';
o = [0;0;0];
xa = [o dcm(:,1)];
xa = xa';
ya = [o dcm(:,2)];
ya = ya';
za = [o dcm(:,3)];
za = za';
plot3(xa(:,1),xa(:,2),xa(:,3), 'r-');hold on; axis equal;
axis([-1 1 -1 1 -1 1]);
xlabel('x');
ylabel('y');
zlabel('z');
plot3(ya(:,1),ya(:,2),ya(:,3), 'g-');
plot3(za(:,1),za(:,2),za(:,3), 'b-');hold off