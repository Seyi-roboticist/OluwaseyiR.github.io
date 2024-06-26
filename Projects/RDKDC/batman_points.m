function final = batman_points()
    % Developer: Seyi R. Afolayan
    % Date: 19/12/2023
    % Credits: Matlab Forum for assistance with piece-wise non-linear equations
    % Method: Path planning points were hardcoded using gradient ascent
    % and descent techniques to plot a shape resembling Batman's logo.

    % Define the x-coordinates of the points outlining the Batman shape.
    % These points are distributed to form the specific outline when plotted.
    x = [-4.92, -5.23, -5.69, -6, -6.3, -6.62, -6.92, -6.46, ...
         -6.31, -5.85, -5.54, -5.35, -4.77, -4.46, -4.15, -3.7, ...
         -3.23, -3.08, -2.8, -2.4, -2, -1.6, -1.2, -0.91, -0.72, ...
         -0.54, -0.3, -0.1, 0.1, 0.3, 0.545, 0.91, 1.6, 2, 2.4, ...
         2.8, 3.12, 3.51, 3.9, 4.3, 4.79, 5.44, 5.82, 6.33, 6.72, ...
         6.97, 6.59, 6.21, 5.7, 5.31, 4.92, 4, 3.58, 3.16, 2.31, ...
         1.89, 1.47, 1.05, 0.631, 0.21, -0.21, -0.63, -1.05, ...
         -1.47, -1.89, -2.74, -3.16, -3.58, -4, -4.92];

    % Define the y-coordinates corresponding to each x-coordinate.
    % These y-coordinates complement the x-coordinates to define the vertical
    % positioning of each point in the Batman shape.
    y = [-2.13, -1.99, -1.74, -1.54, -1.3, -0.98, -0.44, 1.15, ...
         1.3, 1.65, 1.83, 1.91, 2.2, 2.31, 2.41, 2.54, 2.66, 2.69, ...
         1.63, 1.07, 0.86, 0.82, 0.91, 1.73, 2.931, 2.38, 2.25, ...
         2.25, 2.25, 2.85, 2.4, 1.72, 0.82, 0.86, 1.1, 1.62, 2.7, ...
         2.6, 2.5, 2.37, 2.19, 1.9, 1.67, 1.28, 0.84, -0.26, ...
         -1.012, -1.39, -1.74, -1.96, -2.13, -2.46, -1.57, -1.35, ...
         -1.6, -1.93, -1.6, -1.57, -1.791, -2.285, -2.285, -1.79, ...
         -1.58, -1.581, -1.93, -1.35, -1.34, -1.57, -2.46, -2.13];

    % Combine the x and y coordinates into a 2D matrix where each row
    % represents a point in the form [x, y]. This matrix forms the complete
    % set of points defining the outline of the Batman shape.
    final = [x', y']; % Transpose x and y vectors to create 2-column matrix

    % The final matrix is ready for use in plotting or further calculations
    % related to the Batman shape.
end

