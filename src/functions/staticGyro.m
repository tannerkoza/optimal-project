function [start, stop] = staticGyro(gyro, static_thresh)
% DESCRIPTION: This function produces indices corresponding to a static
% period of a gyro dataset..
% PARAMS:
%   - gyro: 3-axis gyroscope data (rad/s)
%   - static_thresh: threshold that defines the norm value for static data
% OUTPUT:
%   - start: beginning of static data index
%   - stop: end of static data index
% AUTHOR: Tanner Koza

%% Initialization

    % Dimension Handling
    [row, col] = size(gyro); 

    if col > row
        gyro = gyro';
    end

%% Bias Calculation
    
    % Step 1
    % DESCRIPTION: This step finds the norm of each time step of gyroscope
    % data. These are compared to a threshold that determines which 
    % indices are static (same as raw gyroscope row indices).

    gyro_norm = sqrt(sum(gyro.^2, 2));

    static_idxs = find(gyro_norm < static_thresh); % static gyro indices

    % Step 2
    % DESCRIPTION: This step determines the indices that represent the
    % bounds of all static data. Then, the indices for the largest group 
    % of static data are extracted.
    
    group_ends = [1; find(diff(static_idxs) ~= 1)];

    a = diff(group_ends);

    max_start = find(a == max(a));

    start = static_idxs(group_ends(max_start) + 1);
    stop = static_idxs(group_ends(max_start + 1));

    if ~isscalar(start) || ~isscalar(stop)
        error(['There is no static period given this threshold. Increase' ...
            ' and evaluate the plot for validity.']);
    end

%% Plotting
    
    lim = 0.5;
    r = stop - start;

    x_lims = [0, stop + (lim * r)];
    y_lims = [0, 10 * static_thresh];

    figure
    
    plot(gyro_norm, '.')
    hold on

    plot(static_idxs, gyro_norm(static_idxs), 'r.')

    plot(start:stop, gyro_norm(start:stop), 'g.')

    yline(static_thresh, 'r', 'LineWidth', 2)

    title('Gyro Norm Static Analysis')
    legend('Raw Gyro', 'Static Gyro', 'Largest Static Group', 'Static Threshold', 'Location','best')
    xlabel('Sample')
    ylabel('Gyro Norm (rad/s)')
    xlim(x_lims)
    ylim(y_lims)


end