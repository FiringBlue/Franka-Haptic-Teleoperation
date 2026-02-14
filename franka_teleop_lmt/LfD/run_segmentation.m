follower_file = '/home/.../TeleopData/follower_1.txt';
out_dir = '/home/.../TeleopData/segments';

out = segment_throw_demo(follower_file, out_dir);  % 不传 opts 就会用默认参数
disp(out);
