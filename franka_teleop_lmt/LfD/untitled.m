M = readmatrix('/home/rmilnuc2/catkin_ws/src/franka_ros/franka_teleop_lmt/TeleopData/SegmentDemo_learned/learned_part1.csv');
z = M(3,:);

figure;
plot(z);
title('learned Z');
xlabel('index');ylabel('z');


A = readmatrix('/home/rmilnuc2/catkin_ws/src/franka_ros/franka_teleop_lmt/TeleopData/SegmentDemo_learned/learned_part1.csv');
figure;
plot(A(3,:));
title('learned Z actually sent to robot');


T = readtable('/home/rmilnuc2/catkin_ws/src/franka_ros/franka_teleop_lmt/TeleopData/z_dump_from_cpp.txt');
plot(T.k, T.z); grid on;
title('Z sequence as read by C++');