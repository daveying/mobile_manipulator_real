% a1 = 2*pi*(rand()-0.5)
% a2 = 2*pi*(rand()-0.5)
% a3 = 2*pi*(rand()-0.5)
a1 = 0;
a2 = 0;
a3 = -pi/4;
%q1 = angle2quat(a1, a2, a3);
q1 = [1 0 0 0];
% q1 = angle2quat(0, pi/2, 0);
% a1 = 2*pi*(rand()-0.5)
% a2 = 2*pi*(rand()-0.5)
% a3 = 2*pi*(rand()-0.5)
a3 = -pi/4;
%q2 = angle2quat(a1, a2, a3);
%q2 = [sqrt(2)/2 sqrt(6)/6 sqrt(6)/6 sqrt(6)/6];
q2 = [1/2 1/2 1/2 1/2];
% q2 = angle2quat(0, -pi/2, 0);
num = 200;
delt_q = q2-q1;
qs = [];
for i = 1:num
    disp(i)
    qq = q1 + i./num.*delt_q
    disp('normalized:')
    qq = quatnormalize(qq)
    qs = [qs; qq];
end
s = size(qs);
for i = 1:s(1)
    draw_dcm(quat2dcm(qs(i,:)));hold on
    pause(0.01)
end
draw_dcm(quat2dcm(q1));hold on
draw_dcm(quat2dcm(q2))