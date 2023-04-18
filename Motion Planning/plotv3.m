function h = plotv3(X0_vec, X1_vec, varargin)
% Function accepts two 3x1 vectors and plots their difference as a line
% segment from X0_vec to X1_vec, with an optional argument for coloring.

% X0_vec: a 3x1 vector
% X1_vec: a 3x1 vector
% varargin: (optional) a color specification, either a MATLAB built-in
% character (e.g. 'r') or a RBG triplet of values between 0 and 1

% usage example:
% plotv3([1; 0; 0], [0; 1; 0]);
% plotv3([1; 0; 0], [0; 1; 0], 'r');
% plotv3([1; 0; 0], [0; 1; 0], rand(1, 3));

% 1997  Louis Whitcomb Created and Written
% 5 Mar 2022 modified Yanzhou Wang 

% check argument dimension
[rows, cols] = size(X0_vec);
if ((rows ~= 3) || (cols ~= 1))
  error('PLOTV3 requires a 3x1 vector argument. Check your dimensions.');
end

% check argument dimension
[rows, cols] = size(X1_vec);
if ((rows ~= 3) || (cols ~= 1))
  error('PLOTV3 requires a 3x1 vector argument. Check your dimensions.');
end

% check number of arguments
if nargin < 2
    error('PLOTV3 requires two 3x1 vector arguments. Check number of inputs.')
end

% assemble a matrix with the two position vectors
m = [X0_vec'; X1_vec'];

% check if the given value for the linestyle exists
if nargin == 3
    colorspec = varargin{1};
else
    % if user does not specify a colorspec, set it to black.
    colorspec = [0, 0, 0];
end


h = plot3(m(:,1), m(:,2), m(:,3), "Color", colorspec, "Marker", 'o');