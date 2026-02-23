%============================================================================
%Name        : run_segmentation.m
%Author      : Wenhao Zhao (wenhao.zhao@tum.de)
%Version     : Feb 2026
%Description : Run the segmentation of a throw demonstration, which is implemented in segment_throw_demo.m. The segmentation results are saved in CSV files 
%============================================================================

follower_file = '/home/.../TeleopData/SegmentDemo/notSegmented/follower_1.txt';
out_dir = '/home/.../TeleopData/SegmentDemo';

out = segment_throw_demo(follower_file, out_dir);  % don't use opt because the segmentation is not based on optimization, but on a heuristic method. 
disp(out);
