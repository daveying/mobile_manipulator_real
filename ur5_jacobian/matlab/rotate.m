q0 = [1 0 0 0];
dcm0 = quat2dcm(q0);
draw_dcm(dcm0);pause

qr = [cos(pi/30) sin(pi/30)*sqrt(3)/3 sin(pi/30)*sqrt(3)/3 sin(pi/30)*sqrt(3)/3];
for i = 1:10
    q0 = quatmultiply(qr,q0);
    draw_dcm(quat2dcm(q0));hold on;pause
end

% dcm_r = quat2dcm(qr);
% draw_dcm(dcm_r);pause
% 
% q1_dcm = quatrotate(qr, dcm0);
% 
% draw_dcm(q1_dcm);