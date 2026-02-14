function out = segment_throw_demo(follower_file, out_dir, opts)
%SEGMENT_THROW_DEMO  Segment a throw demonstration into 3 parts based on
% gripper_event (pulse), pitch_rate_cmd, and speed norm.
%
% OUTPUT:
%   out struct with indices/time windows and file paths.
%
% USAGE:
%   opts = default_segmentation_opts();
%   out  = segment_throw_demo('/path/follower_1.txt', '/path/out', opts);

if nargin < 3 || isempty(opts)
    opts = default_segmentation_opts();
end
if nargin < 2 || isempty(out_dir)
    out_dir = fileparts(follower_file);
end
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

% ---------- Load table ----------
T = readtable(follower_file, 'FileType','text', 'Delimiter','\t');

% Basic sanity checks
required = {'time','Vf_x','Vf_y','Vf_z','Xf_x','Xf_y','Xf_z','pitch_rate_cmd','gripper_event'};
for i=1:numel(required)
    if ~ismember(required{i}, T.Properties.VariableNames)
        error("Missing column: %s", required{i});
    end
end

t   = T.time;
Vf  = [T.Vf_x, T.Vf_y, T.Vf_z];
Xf  = [T.Xf_x, T.Xf_y, T.Xf_z];
pr  = T.pitch_rate_cmd;
ge  = T.gripper_event;

% ---------- Smooth optional (helps thresholds) ----------
if opts.smooth_vel
    Vf_s = movmean(Vf, opts.smooth_win);
else
    Vf_s = Vf;
end
vnorm = sqrt(sum(Vf_s.^2, 2));

% ---------- Detect gripper events (pulse indices) ----------
ge_idx = find(ge > 0.5);
% remove pulses too close (debounce)
if ~isempty(ge_idx)
    keep = true(size(ge_idx));
    for k = 2:numel(ge_idx)
        if (t(ge_idx(k)) - t(ge_idx(k-1))) < opts.event_debounce_s
            keep(k) = false;
        end
    end
    ge_idx = ge_idx(keep);
end

if numel(ge_idx) < 2
    warning("Found %d gripper_event pulses. Segmentation may be unreliable.", numel(ge_idx));
end

% ---------- Find rotation window (pitch active) ----------
rot_mask = abs(pr) > opts.pitch_active_thr;
rot_idx = find(rot_mask);

rot_start = NaN; rot_end = NaN;
if ~isempty(rot_idx)
    rot_start = rot_idx(1);
    rot_end   = rot_idx(end);
end

% ---------- Decide which gripper event is CLOSE vs OPEN ----------
% Heuristic:
% - OPEN typically occurs during/after high-speed throw (vnorm high)
% - CLOSE typically occurs at low speed near grasp
%
% If we have >=2 events, choose "open" as the event with larger vnorm around it.
close_idx = NaN; open_idx = NaN;
if numel(ge_idx) >= 2
    score = zeros(size(ge_idx));
    for k = 1:numel(ge_idx)
        i0 = max(1, ge_idx(k)-opts.event_score_win);
        i1 = min(height(T), ge_idx(k)+opts.event_score_win);
        score(k) = max(vnorm(i0:i1));  % peak speed around event
    end
    [~, k_open] = max(score);
    open_idx = ge_idx(k_open);
    % choose close as earliest event that is not open (usually the first)
    others = ge_idx;
    others(k_open) = [];
    if ~isempty(others)
        close_idx = others(1);
    else
        close_idx = ge_idx(1);
    end
else
    % fallback: if only 1 event, treat it as open if happens at high speed
    if ~isempty(ge_idx)
        open_idx = ge_idx(1);
    end
end

% ---------- Define throw start (Part3 start) ----------
% Find first time vnorm exceeds threshold after rotation end (or after close event)
search_start = 1;
if ~isnan(rot_end)
    search_start = max(search_start, rot_end);
end
if ~isnan(close_idx)
    search_start = max(search_start, close_idx);
end

throw_start = find(vnorm > opts.throw_speed_start_thr & ( (1:height(T))' >= search_start), 1, 'first');

if isempty(throw_start)
    warning("Could not detect throw_start with threshold %.3f. Using rot_end or close_idx.", opts.throw_speed_start_thr);
    throw_start = search_start;
end

% ---------- Define open time & latency compensated time ----------
open_time = NaN;
open_time_latency = NaN;
if ~isnan(open_idx)
    open_time = t(open_idx);
    open_time_latency = open_time + opts.gripper_latency_s; % approximate "actual release"
end

% ---------- Build segment indices ----------
% Part1: start -> close event (grasp command). If close is unknown, use start->rot_start-1
p1_start = 1;
if ~isnan(close_idx)
    p1_end = close_idx;
elseif ~isnan(rot_start)
    p1_end = max(1, rot_start-1);
else
    p1_end = min(height(T), round(height(T)*0.3));
end

% Part2: after close -> just before throw_start (includes move to pre-throw + rotation)
p2_start = min(height(T), p1_end+1);
p2_end   = max(p2_start, throw_start-1);

% Part3: throw_start -> open event (or end)
p3_start = throw_start;
if ~isnan(open_idx)
    p3_end = open_idx;
else
    p3_end = height(T);
end

% Clip to valid bounds
[p1_start,p1_end,p2_start,p2_end,p3_start,p3_end] = clip_segments(height(T), p1_start,p1_end,p2_start,p2_end,p3_start,p3_end);

% ---------- Export segments ----------
base = string(get_filename_stem(follower_file));   % 已经不含 .txt 了

part1_file = fullfile(out_dir, base + "_part1.csv");
part2_file = fullfile(out_dir, base + "_part2.csv");
part3_file = fullfile(out_dir, base + "_part3.csv");

writetable(T(p1_start:p1_end, :), part1_file);
writetable(T(p2_start:p2_end, :), part2_file);
writetable(T(p3_start:p3_end, :), part3_file);

% ---------- Summary (robust across MATLAB versions) ----------
summary_file = fullfile(out_dir, base + "_seg_summary.csv");

S = table();
S.file = string(follower_file);

S.p1_i0 = p1_start; S.p1_i1 = p1_end; S.p1_t0 = t(p1_start); S.p1_t1 = t(p1_end);
S.p2_i0 = p2_start; S.p2_i1 = p2_end; S.p2_t0 = t(p2_start); S.p2_t1 = t(p2_end);
S.p3_i0 = p3_start; S.p3_i1 = p3_end; S.p3_t0 = t(p3_start); S.p3_t1 = t(p3_end);

S.close_i = close_idx;
S.close_t = NaN; if ~isnan(close_idx), S.close_t = t(close_idx); end

S.open_i = open_idx;
S.open_t = NaN;  if ~isnan(open_idx),  S.open_t  = t(open_idx);  end

S.open_t_plus_latency = open_time_latency;

S.rot_i0 = rot_start;
S.rot_t0 = NaN; if ~isnan(rot_start), S.rot_t0 = t(rot_start); end

S.rot_i1 = rot_end;
S.rot_t1 = NaN; if ~isnan(rot_end), S.rot_t1 = t(rot_end); end

S.throw_i0 = throw_start;
S.throw_t0 = t(throw_start);

writetable(S, summary_file);


% Return outputs
out = struct();
out.part1_file = part1_file;
out.part2_file = part2_file;
out.part3_file = part3_file;
out.summary_file = summary_file;
out.close_idx = close_idx;
out.open_idx = open_idx;
out.throw_start = throw_start;
out.rot_start = rot_start;
out.rot_end = rot_end;
out.opts = opts;

% ---------- Optional plot ----------
if opts.plot
    figure('Name', 'Segmentation Debug'); clf; hold on; grid on;
    plot(t, vnorm, 'LineWidth', 1.2);
    yline(opts.throw_speed_start_thr, '--');
    xlabel('time [s]'); ylabel('||Vf|| [m/s]');
    title('Throw segmentation (speed norm)');

    % Mark segments
    xline(t(p1_end), 'k-', 'Part1 end');
    xline(t(p2_end), 'k-', 'Part2 end');
    if ~isnan(close_idx), xline(t(close_idx), 'g-', 'CLOSE'); end
    if ~isnan(open_idx),  xline(t(open_idx),  'r-', 'OPEN'); end
    if ~isnan(rot_start), xline(t(rot_start),'c--','rot start'); end
    if ~isnan(rot_end),   xline(t(rot_end),  'c--','rot end'); end
    xline(t(throw_start), 'm--', 'throw start');
    legend('||Vf||','thr','Location','best');
end

fprintf("Saved:\n  %s\n  %s\n  %s\nSummary:\n  %s\n", part1_file, part2_file, part3_file, summary_file);

end

% ---------------- helper opts ----------------
function opts = default_segmentation_opts()
opts = struct();
opts.pitch_active_thr = 1e-3;        % rad/s; pitch considered active above this
opts.throw_speed_start_thr = 0.22;   % m/s; start of throw acceleration
opts.gripper_latency_s = 0.10;       % seconds; approximate open latency
opts.event_debounce_s = 0.10;        % seconds; ignore pulses closer than this
opts.event_score_win = 20;           % samples around event to score (if ~1kHz, adjust)
opts.smooth_vel = true;
opts.smooth_win = 10;               % moving mean window for velocity
opts.plot = true;
end

function [a,b,c,d,e,f] = clip_segments(N, a,b,c,d,e,f)
a = max(1, min(N, a)); b = max(1, min(N, b));
c = max(1, min(N, c)); d = max(1, min(N, d));
e = max(1, min(N, e)); f = max(1, min(N, f));
% ensure monotonic ordering
if b < a, b = a; end
if d < c, d = c; end
if f < e, f = e; end
end

function stem = get_filename_stem(p)
[~, stem, ~] = fileparts(p);   % stem 不含 .txt
end


function y = iff_nan(idx, nanv, val)
if isnan(idx)
    y = nanv;
else
    y = val;
end
end
