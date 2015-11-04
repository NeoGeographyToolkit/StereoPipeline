function main(do_find, dirs, pitches)

   % if do_find is 1, we actually find and save the CCDs,
   % otherwise we just examine the current averaged disparties.

   if nargin == 2
      pitches = [];
   end
   
   ax={};
   ay={};
   for i=1:length(dirs)
      dir = dirs{i};
      ax0=[dir '/dx.txt'];
      ay0=[dir '/dy.txt'];
      ax = [ax, ax0];
      ay = [ay, ay0];
   end
   
   do_plot(do_find, ax, pitches, 1);
   title('x');
   
   do_plot(do_find, ay, pitches, 2); 
   title('y');

function dx = scale_by_pitch(dx, pitch)
   % compensate for the fact that ccd artifacts are spaced closer
   % for larger pitch
   base_pitch = 8.0000000000e-03;
   I = (base_pitch/pitch)*(1:(10*length(dx)));
   J=find(I > length(dx));
   I = I(1:(J(1)-1));
   dx = interp1(1:length(dx), dx', I, 'linear')';
   
function do_plot(do_find, d, pitches, fig)
   
   X = [];
   for i=1:length(d)
      dxf=d{i};
      if exist(dxf, 'file') == 2
         %disp(sprintf('Loading %s', dxf));
      else
         disp(sprintf('Missing file: %s', dxf));
         continue;
      end
      dx = load(dxf);

      if length(pitches) >= i
         dx = scale_by_pitch(dx, pitches(i));
      end
      
      [m, n] = size(X);
      % Deal with size mis-matches by either
      % growing X or dx.
      if m > 0
         if length(dx) < m
            dx = [dx' zeros(1, m - length(dx))]';
         elseif m < length(dx)
            m1 = length(dx);
            Y = zeros(m1, n);
            Y(1:m, 1:n) = X;
            X = Y;
            [m, n] = size(X);
         end
      end

      X = [X dx];
      
   end
   
   X = X';

   [m, n] = size(X);
   %disp(sprintf('size is %d %d', m, n));
   
   for r=1:m

      % Stay away from boundary.
      bdval = 300; % Must tweak this!!!
      c0 = round(n/3);
      c1 = n - c0;
      cs = 1;
      for c=1:c0
         if X(r, c) == 0
            cs = max(cs, c);
         end
      end

      ce = n;
      for c=(n-c0):n
         if X(r, c) == 0
            ce = min(ce, c);
         end
      end
      
      cs = cs + bdval;
      ce = ce - bdval;
      for c=1:cs
         X(r, c) = NaN;
      end
      for c=ce:n
         X(r, c) = NaN;
      end
      
   end
   
   wid = 35;

   A = [];
   figure(fig); clf; hold on;
   colors=['b', 'r', 'g', 'c', 'k', 'b', 'r', 'g', 'c', 'b', 'r', 'g', 'k', 'c'];
   q=0;
   s=1.0; % cutoff
   t=0.0;
   if do_find
      t = 0.5; % shift when plotting
   end
   
   for r=1:m
      q=q+1;
      q = rem(q-1, length(colors)) + 1;
      if r <= length(d) 
         disp(sprintf('doing %s', d{r}));
      end

      Y = X(r, :)-find_moving_avg(X(r, :));
      X(r, :) = Y;
      r2 = rem(r-1, length(colors))+1;
      plot(Y + t*(r+1), colors(r2));         
   end

   [m, n] = size(X);
   Z = zeros(1, n);
   for c=1:n
      sum = 0;
      num = 0;
      for r=1:m
         if ~isnan(X(r, c))
            sum = sum + X(r, c);
            num = num + 1;
         end
      end
      if num > 0
         Z(1, c) = sum/num;
      end
   end

   if do_find ~= 0
      find_ccds_aux(Z, fig)
   end
   
function b = split(a)

   % split by space character
   b={};

   c = '';
   for i=1:length(a)
      if isspace(a(i))
         if length(c) ~= 0
            b = [b, c];
            c = '';
         end
      else
         c = [c, a(i)];
      end
   end

   if length(c) ~= 0
      b = [b, c];
   end
   