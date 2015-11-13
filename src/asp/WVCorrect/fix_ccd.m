% Given a plotted polygonal line, move its vertices by clicking
% on the old location and the new location.

function A = fix_ccd(A, fig)
   figure(fig);
   [x, y] = ginput(1);
   B = abs(A(1, :) - x);
   I = find(B == min(B));
   i = I(1);
   A(1, i) = x;
   A(2, i) = y;
   figure(fig); hold on;
   plot(A(1, :), A(2, :), 'g')
   plot(x, y, 'r*')
   disp(sprintf('value is %g', y));