function h = plotFrames3D(p, fa)
% Plot extruded 3D frame-shaped pegs with thickness

    colPegs = [];
    for p_ = 1:numel(p)
        colPegs = [colPegs; p(p_).color];
    end
  
         

    if nargin < 3
        fa = 0.6;
    end

    % Base frame shape (2D XY)
    frame_2D = [-0.1 0.06; -0.1 0.1; 0.1 0.1; 0.1 0.06;
                -0.06 0.06; -0.06 -0.06; 0.1 -0.06; 0.1 -0.1;
                -0.1 -0.1; -0.1 -0.06] ;

    n = size(frame_2D,1);  % number of edge points
    thickness = 0.02;

    % Extrude to 3D
    bottom = [frame_2D, zeros(n,1)];
    top    = [frame_2D, thickness * ones(n,1)];
    vertices = [bottom; top]';

    % Apply rotation for Gazebo compatibility
    vertices = rotz(180) * vertices;

    % Define face list
    faces = {};

    % Bottom face
    faces{end+1} = 1:n;

    % Top face
    faces{end+1} = 2*n:-1:(n+1);

    % Side faces
    for i = 1:n
        i_next = mod(i, n) + 1;
        faces{end+1} = [i, i_next, i_next + n, i + n];
    end

    % Plot each transformed frame
    for m = 1:length(p)
        R = p(m).A;
        t = p(m).b;

        verts = R * vertices + t;

        % Use patch for each face
        for f = 1:length(faces)
            h(m,f) = patch('Vertices', verts', ...
                           'Faces', faces{f}, ...
                           'FaceColor', colPegs(m,:), ...
                           'EdgeColor', 'k', ...
                           'FaceAlpha', fa, ...
                           'LineWidth', 1);
        end
    end
end
