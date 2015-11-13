%!rsync -avz  oalexan1@m.ndc.nasa.gov:projects/ccd/\* /home/oalexan1/projects/ccd 2>/dev/null

%function main(tdi)
%   
%   do_plot(tdi, 'x');
%   do_plot(tdi, 'y');
%   
%function do_plot(tdi, dir)   
%
%   prefix='w1fwd';
%   file=sprintf('ccd%s_%s_tdi%d.txt', dir, prefix, tdi);
%   disp(sprintf('Loading %s', file));
%   A = load(file);
%
%   prefix='w1bwd';
%   file=sprintf('ccd%s_%s_tdi%d.txt', dir, prefix, tdi);
%   disp(sprintf('Loading %s', file));
%   B = load(file);
%
%   figno = 1;
%   if dir ~= 'x'
%      figno = 2;
%   end
%   
%   figure(figno); clf; hold on;
%   title(dir);
%   plot(A(1, :), A(2, :), 'b');
%   plot(B(1, :), B(2, :), 'r');
%   
%   return
%
   s='x';
   prefix=sprintf('ccd%s_w1fwd', s);
   %A=[48, 56, 64, 8];
   A = [32, 33];
   colors=['b', 'r', 'g', 'k', 'm', 'c', 'y'];
   L={};
   F={};
   figure(4); clf; hold on; %clf; hold on;
   for i=1:length(A)
      file=sprintf('%s_tdi%d.txt', prefix, A(i));
      disp(sprintf('Loading %s', file));
      C = load(file);
      [m, n] = size(C);
      disp(sprintf('size is %d %d', m, n));
      plot(C(1, :), C(2, :), [colors(i), '*']);
      plot(C(1, :), C(2, :), [colors(i)]);
      F=[F, C];
      L=[L, file];
   end
   A = load(sprintf('avg%s.txt', s));
   plot(A+0., 'g')
   
   %legend(L);


   %figure(4); clf; hold on;
   %plot(abs(F{3}-F{2}), 'b');
   %plot(abs(F{3}-F{4}), 'r');
   %
   %G = 0.5*(F{1}+F{3});
   %format long g
   %file=sprintf('%s_tdi%d.txt', prefix, A(2));
   %disp(sprintf('saving %s', file));
   %save(file, '-ascii', '-double', 'G');
