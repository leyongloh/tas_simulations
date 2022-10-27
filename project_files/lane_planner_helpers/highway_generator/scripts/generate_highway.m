%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @info: script to generate highway ground texture for gazebo including
% option to export post coordinates
%
% @author: Fabian Zillenbiller
%
% @contact: ga27seq@tum.de
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% options
export_figure = false; % pls move png to highway/materials/texture if true (high computation time) 

create_plot = true;
plot_grid = false; % use to scale exported picture in gazebo

plot_obj = false; % show obj positions in plot
export_obj = true; % export objects to yaml file

%% initialize variables
X.ds = 1; % do not change

X.lane_length = 500;
X.curve_diam = 200;

X.rad_curve = X.curve_diam / 2;

X.scale = 1 / 4; 
map_size = 740;

%% get lane points
road_c = get_coordinates(0, X);
road_1i = get_coordinates(-3.75, X);
road_2i = get_coordinates(-7.5, X);
road_3i = get_coordinates(-8.25, X);
road_4i = get_coordinates(-10.75, X);

road_1o = get_coordinates(3.75, X);
road_2o = get_coordinates(7.5, X);
road_3o = get_coordinates(8.25, X);
road_4o = get_coordinates(10.75, X);

%% get obj points
post = get_obj_coord(X, 50, "1234", "both", [-11.75, 11.75]);
straight_rail = get_obj_coord(X, 9, "13", "both", [-11.25, 11.25]);
inner_rail = get_obj_coord(X, 9, "24", "inner", [-11.25, 11.25]);
outer_rail = get_obj_coord(X, 9, "24", "outer", [-11.25, 11.25]);

%% plot
if create_plot
    plotbox = [[-map_size; map_size], [-map_size; -map_size], [map_size; -map_size], [map_size; map_size]] * X.scale;


    green = [162, 173, 0] / 255;
    lgray = [204, 204, 204] / 255;
    mgray = [185, 185, 185] / 255;
    dgray = [153, 153, 153] / 255;
    width = 0.01;

    figure
    hold on
    set(gca, 'Color', green)
    fill(plotbox(1, :), plotbox(2, :), green, 'LineStyle', 'none')
    fill(road_4o(1, :), road_4o(2, :), lgray, 'LineStyle', 'none')
    fill(road_3o(1, :), road_3o(2, :), mgray, 'LineStyle', 'none')
    fill(road_2o(1, :), road_2o(2, :), dgray, 'LineStyle', 'none')
    fill(road_2i(1, :), road_2i(2, :), mgray, 'LineStyle', 'none')
    fill(road_3i(1, :), road_3i(2, :), lgray, 'LineStyle', 'none')
    fill(road_4i(1, :), road_4i(2, :), green, 'LineStyle', 'none')


    plot(road_2o(1, :), road_2o(2, :), 'w', 'LineWidth', width)
    plot(road_1o(1, :), road_1o(2, :), '--w', 'LineWidth', width)
    plot(road_c(1, :), road_c(2, :), 'w', 'LineWidth', width)
    plot(road_1i(1, :), road_1i(2, :), '--w', 'LineWidth', width)
    plot(road_2i(1, :), road_2i(2, :), 'w', 'LineWidth', width)

    if plot_obj
        scatter(post(1, :), post(2, :), '+k')
        scatter(post(4, :), post(5, :), '+k')

        scatter(straight_rail(1, :), straight_rail(2, :), '+b')
        scatter(straight_rail(4, :), straight_rail(5, :), '+b')

        scatter(inner_rail(1, :), inner_rail(2, :), '+b')
        scatter(outer_rail(1, :), outer_rail(2, :), '+b')
    end

    if plot_grid
        plot([3, -3], [0, 0], 'k', 'LineWidth', width)
        plot([0, 0], [3, -3], 'k', 'LineWidth', width)

        plot([3, -3], [1, 1], 'k', 'LineWidth', width)
        plot([1, 1], [3, -3], 'k', 'LineWidth', width)

        plot([3, -3], [2, 2], 'k', 'LineWidth', width)
        plot([2, 2], [3, -3], 'k', 'LineWidth', width)

        plot([3, -3], [3, 3], 'k', 'LineWidth', width)
        plot([3, 3], [3, -3], 'k', 'LineWidth', width)

        plot([3, -3], [-1, -1], 'k', 'LineWidth', width)
        plot([-1, -1], [3, -3], 'k', 'LineWidth', width)

        plot([3, -3], [-2, -2], 'k', 'LineWidth', width)
        plot([-2, -2], [3, -3], 'k', 'LineWidth', width)

        plot([3, -3], [-3, -3], 'k', 'LineWidth', width)
        plot([-3, -3], [3, -3], 'k', 'LineWidth', width)
    end

    set(gca, 'XTick', [], 'YTick', [])
    set(gca, 'visible', 'off')
    hold off
    axis('equal')
end

%% export data
if export_figure
    export_fig Highway_final.png -m45
end
if export_obj
    cd ..
    cd("config")
    yaml.WriteYaml('post.yaml', post); %file names must match .obj path
    yaml.WriteYaml('guardrail_straight.yaml', straight_rail);
    yaml.WriteYaml('guardrail_inner_curve.yaml', inner_rail);
    yaml.WriteYaml('guardrail_outer_curve.yaml', outer_rail);
    cd ..
    cd("scripts")
end

%% get coordinates
function x = get_x(phi, X, off, curve_id)
if curve_id == 1
    x = X.lane_length / 2 + (X.rad_curve + off) * cos(phi*pi/180);
else
    x = -X.lane_length / 2 + (X.rad_curve + off) * cos(phi*pi/180);
end
end

function y = get_y(phi, X, off)
y = (X.rad_curve + off) * sin(phi*pi/180);
end

%%
function obj_coord = get_obj_coord(X, obj_interval, zones, area, offset)

dphi = (X.ds * 180) / (pi * (X.rad_curve + offset(1)));

obj_coord = zeros(6, 1);


if contains(zones, "1")
    % lane 1
    x = -X.lane_length / 2;
    y = -X.rad_curve - offset(1);
    while x < X.lane_length / 2 - obj_interval * 0.8
        last_obj = norm([x, y]-[double(obj_coord(1, end)), double(obj_coord(2, end))]);
        if obj_interval < last_obj
            obj_coord = [obj_coord, [; ...
                x; ...
                -X.rad_curve - offset(1); ...
                pi; ...
                x; ...
                -X.rad_curve - offset(2); ...
                0; ...
                ], ...
                ];
        end
        x = x + X.ds;
    end
end

if contains(zones, "2")
    % curve 1
    phi = 270;
    while phi < 360 - dphi * 5
        x = get_x(phi, X, offset(1), 1);
        y = get_y(phi, X, offset(1));

        last_obj = norm([x, y]-[double(obj_coord(1, end)), double(obj_coord(2, end))]);
        if obj_interval < last_obj
            obj_coord = [obj_coord, [; ...
                get_x(phi, X, offset(1), 1); ...
                get_y(phi, X, offset(1)); ...
                deg2rad(phi-90); ...
                get_x(phi, X, offset(2), 1); ...
                get_y(phi, X, offset(2)); ...
                deg2rad(phi+90); ...
                ], ...
                ];
        end
        phi = phi + dphi;
    end

    phi = 0;
    while phi < 90 - dphi * 5
        x = get_x(phi, X, offset(1), 1);
        y = get_y(phi, X, offset(1));
        last_obj = norm([x, y]-[double(obj_coord(1, end)), double(obj_coord(2, end))]);

        if obj_interval < last_obj
            obj_coord = [obj_coord, [; ...
                get_x(phi, X, offset(1), 1); ...
                get_y(phi, X, offset(1)); ...
                deg2rad(phi-90); ...
                get_x(phi, X, offset(2), 1); ...
                get_y(phi, X, offset(2)); ...
                deg2rad(phi+90); ...
                ], ...
                ];
        end
        phi = phi + dphi;
    end
end

if contains(zones, "3")
    % lane 2
    x = X.lane_length / 2;
    y = X.rad_curve + offset(1);
    while -X.lane_length / 2 + obj_interval * 0.8 < x
        last_obj = norm([x, y]-[double(obj_coord(1, end)), double(obj_coord(2, end))]);
        if obj_interval < last_obj
            obj_coord = [obj_coord, [; ...
                x; ...
                X.rad_curve + offset(1); ...
                0; ...
                x; ...
                X.rad_curve + offset(2); ...
                pi; ...
                ], ...
                ];
        end
        x = x - X.ds;
    end
end

if contains(zones, "4")
    % curve 2
    phi = 90;
    while phi < 270 - dphi * 5
        x = get_x(phi, X, offset(1), 2);
        y = get_y(phi, X, offset(1));

        last_obj = norm([x, y]-[double(obj_coord(1, end)), double(obj_coord(2, end))]);
        if obj_interval < last_obj
            obj_coord = [obj_coord, [; ...
                get_x(phi, X, offset(1), 2); ...
                get_y(phi, X, offset(1)); ...
                deg2rad(phi-90); ...
                get_x(phi, X, offset(2), 2); ...
                get_y(phi, X, offset(2)); ...
                deg2rad(phi+90); ...
                ], ...
                ];
        end
        phi = phi + dphi;

    end
end

obj_coord = double(obj_coord(:, 2:end)); % delete first and last element
if area == "inner"
    obj_coord = obj_coord(1:3, :);
elseif area == "outer"
    obj_coord = obj_coord(4:6, :);
end

[n, ~] = size(obj_coord);
obj_coord = obj_coord .* repmat([X.scale; X.scale; 1], n/3, 1);
end

%%
function road_center = get_coordinates(offset, X)
dphi = (X.ds * 180) / (pi * (X.rad_curve + offset));

road_center = [];

% lane 1
x = -X.lane_length / 2;
while x < X.lane_length / 2 - X.ds / 2
    road_center = [road_center, [x; -X.rad_curve - offset]];
    x = x + X.ds;
end

% curve 1
phi = 270;
while phi < 360 - dphi / 2
    x = X.lane_length / 2 + (X.rad_curve + offset) * cos(phi*pi/180);
    y = (X.rad_curve + offset) * sin(phi*pi/180);
    road_center = [road_center, [x; y]];
    phi = phi + dphi;
end

phi = 0;
while phi < 90 - dphi / 2
    x = X.lane_length / 2 + (X.rad_curve + offset) * cos(phi*pi/180);
    y = (X.rad_curve + offset) * sin(phi*pi/180);
    road_center = [road_center, [x; y]];
    phi = phi + dphi;
end

% lane 2
x = X.lane_length / 2;
while -X.lane_length / 2 + X.ds / 2 < x
    road_center = [road_center, [x; X.rad_curve + offset]];
    x = x - X.ds;

end

% curve 2
phi = 90;
while phi < 270 - dphi / 2
    x = -X.lane_length / 2 + (X.rad_curve + offset) * cos(phi*pi/180);
    y = (X.rad_curve + offset) * sin(phi*pi/180);
    road_center = [road_center, [x; y]];
    phi = phi + dphi;

end

% close loop
road_center = [road_center, [road_center(1, 1); road_center(2, 1)]];

% scale coordinates
road_center = road_center * X.scale;
end
