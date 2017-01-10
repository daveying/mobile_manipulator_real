%% simulation

duration = 40;%seconds
freq = 100;%Hz
step = 1/freq;
t = 0:step:duration;%system time

w_tool_quat = [1 0 0 0];
w_tool_rm = (quat2dcm(w_tool_quat))';%rotation matrix: tool frame respect to world frame
w_goal_quat = [1/2 1/2 1/2 0];
w_goal_rm = (quat2dcm(w_goal_quat))';%rotation matrix: goal frame respect to world frame
C_angular = [];
figure;
sum_ang = 0;
for i = t
%     w_goal_quat = [1/2 1/2 1/2 0] + random('norm', zeros(1,4), [0.002 0.002 0.002 0.002]);
%     w_goal_rm = (quat2dcm(w_goal_quat))';
    tool_goal_rm = w_tool_rm' * w_goal_rm;%rotation matrix: goal frame respect to tool frame
    tool_goal_quat = dcm2quat(tool_goal_rm');
    tool_goal_quat = quatnormalize(tool_goal_quat);
    angle = 2*acos(tool_goal_quat(1));
    axis = tool_goal_quat(2:4)./sin(angle/2);
    t_wxyz = 0.8 * angle * axis;%P control, get the angular velocities respect to tool frame
    w_axis = w_tool_rm * axis';%get the angular velocities respect to world frame
    %drive the tool frame
    sum_ang = sum_ang + angle;
    qd = [cos((0.3*angle+0.01*sum_ang)*step/2) sin((0.3*angle+0.01*sum_ang)*step/2)*w_axis'];
    C_angular = [C_angular 0.08*angle];
    w_tool_quat = quatmultiply(qd, w_tool_quat);
    w_tool_quat = w_tool_quat + random('norm', zeros(1,4), [0.002 0.002 0.002 0.002]);
    w_tool_rm = (quat2dcm(w_tool_quat))';%rotation matrix: tool frame respect to world frame
    draw_dcm(quat2dcm(w_tool_quat))
    pause(0.000001)
end
figure
plot(t,C_angular);


