function plot_ccds(do_find, is_forward, prefix, I)
   % Plot ccds with given scan direction with given prefix.
   % I are their indicies in 'scandir.txt'. 
   % If is_forward=0, that means reverse.
   % The 'scandir.txt' file is obtained by running
   % gen_scandir.pl.
   % The 'good' file, if it exists, must be written by hand,
   % each line has a good run directory, that is, it has
   % nice dx.txt and dy.txt. 
   
   file = sprintf('scandir_%s.txt', prefix);
   disp(sprintf('Loading %s', file));

   suff = 'fwd';
   if ~ is_forward
      suff = 'bwd';
   end
   goodfile = sprintf('good_%s_%s.txt', suff, prefix);
   good_exists = 0;
   if exist(goodfile, 'file') == 2
      good_exists = 1;
      disp(sprintf('Getting good dirs from %s', goodfile));
      good = textread(goodfile, '%s');
   else
      disp(sprintf('File %s does not exist', goodfile));
      good = {};
   end
   
   A=load(file);
   [m, n] = size(A);

   dirs={};
   pitches = [];
   
   if length(I) == 0
      I = 1:m; % do all runs
   end
   for i = I
      if i > m
         if length(I) > 1
            disp(sprintf('Out of bounds, %d', i));
         end
         continue;
      end
      d=A(i, 1);
      a=A(i, 2);
      pa = A(i, 3); % pitch
      b=A(i, 4);
      pb = A(i, 5); % pitch

      pitch0 = 8.000000000000000e-03;
      
      if a == is_forward % & pa == pitch0
         dir=sprintf('%s_%d/runv%s_%d_flip%d', prefix, d, prefix, d, 0);
         [U, V] = ind2sub(size(good),find(cellfun(@(x)strcmp(x,dir),good)));
         if length(U) > 0 | ~good_exists
            dirs=[dirs, dir];
            pitches = [pitches, pa];
         end
      end
      
      if b == is_forward % & pb == pitch0
         dir=sprintf('%s_%d/runv%s_%d_flip%d', prefix, d, prefix, d, 1);
         [U, V] = ind2sub(size(good),find(cellfun(@(x)strcmp(x,dir),good)));
         if length(U) > 0 | ~good_exists
            dirs=[dirs, dir];
            pitches = [pitches, pb];
         end
      end
      
   end

   if length(dirs) > 0
      find_ccds(do_find, dirs, pitches);
   end

   pause(1)
   commandwindow % move the focus back to the command window