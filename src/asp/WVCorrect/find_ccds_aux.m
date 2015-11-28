function find_ccds_aux(mean_disparity, fig)
   % 

   n = length(mean_disparity);

   % Access the existing figure and draw the mean disparity line
   figure(fig); hold on;
   plot(mean_disparity, 'm', 'LineWidth',3); 
   
   % Call function to detect jumps in the disparity vector
   detected_jumps=find_ccds_aux2(mean_disparity);

   [m, num_jumps] = size(detected_jumps)
   if (num_jumps == 0)
     disp('No CCD jumps detected!')
     return % Don't need to write output if we did not find anything.
   end

   I0=1:(n);
   jump_indices    = detected_jumps(1, :);
   jump_magnitudes = detected_jumps(2, :);
   
   WIDTH  = 60;
   period = 705;
   shift  = -80;
   [jump_indices, jump_magnitudes] = sparse_ccds(WIDTH, period, shift, n, jump_indices, jump_magnitudes);

   plot(jump_indices, mean_disparity(jump_indices), 'b*'); % Draw blue *'s
   plot(jump_indices, jump_magnitudes, 'b'); % Draw blue lines?

   %[period, shift] = find_true_period(jump_indices, jump_magnitudes);
   
%   if fig == 1
%      figure(fig+10);
%      clf; hold on;
%      plot(diff(jump_indices), 'b');
%      plot(1000*jump_magnitudes, 'r');
%   end
   
   format long g;
   T = [jump_indices' jump_magnitudes']';
   if fig == 1
      file='ccdx.txt';
      file2='avgx.txt';
   else
      file='ccdy.txt';
      file2='avgy.txt';
   end

   mean_disparity = mean_disparity';
   
   % Write the output files
   disp(sprintf('saving %s', file));
   disp(sprintf('saving %s', file2));
   save(file,  '-ascii', '-double', 'T');
   save(file2, '-ascii', '-double', 'mean_disparity');
   

function [period, shift] = find_true_period(jump_indices, jump_magnitudes)
   % The true period is associated with the hugest hump
   n = length(jump_indices);
   Q = zeros(n-1, 1);
   for i=1:(n-1)
      Q(i) = abs(jump_magnitudes(i)) + abs(jump_magnitudes(i+1));
   end
   I = find(Q == max(Q));
   i = I(1);
   period = jump_indices(i+1) - jump_indices(i);
   shift = jump_indices(i) - i*period;
   while shift > 0
      shift = shift - period;
   end

   plot(jump_indices(i), jump_magnitudes(i), 'g*');
   disp(sprintf('--period and shift %g %g', period, shift));
   
   
function [jump_indices, jump_magnitudes] = sparse_ccds(wid, period, shift, n, jump_indices, jump_magnitudes)
   
   % Keep one CCD per period
   P = period*(1:n) + shift;
   I = find(P <= n);
   P = P(I);
   I=[];
   for i=1:length(P)
      J = find( jump_indices >= P(i) - wid & jump_indices <= P(i) + wid);
      if length(J) == 0
         continue
      end
      K = find( abs(jump_magnitudes(J)) == max(abs(jump_magnitudes(J))) );
      I = [I, J(K(1))];
   end
   jump_indices    = jump_indices(I);
   jump_magnitudes = jump_magnitudes(I);
   
function U = find_ccds_aux2(mean_disparity)
   % Detect local maxima of a diff function in the input vector
   % - Returns indices and values in a matrix

   % Hard coded debug figure
   do_plot = 0; figno=0;
   SHIFT=0;
   if do_plot
      figure(figno); clf; hold on;
   end
   
   %mean_disparity = find_avg(A, col);
   %Bt=mean_disparity';
   %save('Bm.txt', '-ascii', '-double', 'Bt');
   %return
   
   %mean_disparity0 = find_avg(A0, col);
   
   WIDTH = 35;
   
   % Compute some sort of diff measure
   Q = mean_disparity*0 + NaN; % Generate a NaN vector matching the disparity vector
   for i=1:length(mean_disparity)
      %if i-WIDTH >= 1 & i+WIDTH<= length(S)
      if (i-WIDTH >= 1) & (i+2*WIDTH <= length(mean_disparity)) % 3*WIDTH region 1/3 centered on i
         % Document this!!!
         % To do: Improve the accuracy by doing slopes with least squares
         Q(i) =   1.5*(mean_disparity(i+  WIDTH) - mean_disparity(i      )) ...
                - 0.5*(mean_disparity(i+2*WIDTH) - mean_disparity(i-WIDTH));
         %Q(i) = WIDTH*(2*S(i) - S(i-WIDTH) - S(i + WIDTH))/2; 
      end
   end
   
   % To do: Use some other criterion
   cutoff     = 0.01; % temporary!!! % CCD artifacts must be less than this
   max_cutoff = 1.5; % no ccd artifacts more than that are expected
   wid2       = ceil(1.5*WIDTH); % leave only largest maxium in length wid2 on each side
   
   q_indices = [];
   q_maxima = [];
   for i=1:length(Q)
      if isnan(Q(i)) 
         continue
      end
      
      % Only process regions in a certain range
      if abs(Q(i)) < cutoff | abs(Q(i)) > max_cutoff
         continue
      end
      
      % Skip locations near the boundary
      if i-wid2 < 1 | i + wid2 > length(Q)
         continue
      end
      
      % Look at the nearby Q values; if any are greater than the current location
      %   don't retain this location.  This is to only keep local maxima?
      % To do: This needs more thinking, can result in a chain
      % reaction eliminating too many artifacts.
      is_good = 1;
      for l = (i-wid2):(i+wid2)
         if abs(Q(i)) < abs(Q(l))
            is_good = 0;
         end
      end
      
      if is_good % Record the point index and Q value
         q_indices = [q_indices, i];
         q_maxima  = [q_maxima,  Q(i)];
      end
   end
   
   w2 = floor(WIDTH/2);
   
   %G = G + w2; % to be at center of CCD defect region
   format long g
   %[G'+w2 V']'
   
   % Pack centered indices and the value into the output vector
   U = [q_indices'+w2+SHIFT   q_maxima']';
   
   
   %return
   %
   %%
   %%plot(detected_jumps(1, K0), 0*mean_disparity(detected_jumps(1, K0)), 'b*')
   %%plot(sx1(1, K1)+s, 0*dx1(sx1(1, K1)), 'r*')
   %
   %%return
   %
   %
   %
   %%J1=find(I1+s > bx & I1+s < ex);
   %%K1=find(sx1(1, :) + s > bx & sx1(1, :) + s < ex);
   %%
   %%plot(I1(J1)+s, -dx1(J1)', 'r');
   %%plot(sx1(1, K1)+s, -dx1(sx1(1, K1)), 'r*')
   %%
   %%plot(detected_jumps(1, K0), 0*mean_disparity(detected_jumps(1, K0)), 'b*')
   %%plot(sx1(1, K1)+s, 0*dx1(sx1(1, K1)), 'r*')
   %
   %jump_indices = detected_jumps(1, :);
   %P1 = sx1(1, :);
   %Z = zeros(length(jump_indices), length(P1));
   %for l=1:length(jump_indices)
   %   %disp(sprintf('l=%d', l));
   %   for t = 1:length(P1)
   %      P1_shift = P1 - P1(t)+jump_indices(l);
   %      for v = 1:length(jump_indices)
   %         T = find(P1_shift >= jump_indices(v) - d & P1_shift <= jump_indices(v) + d);
   %         Z(l, t) = Z(l, t) + length(T);
   %         %      if v == l
   %         %         disp(sprintf('%d %d %d', t, l, T));
   %         %      end
   %      end
   %   end
   %end
   %
   %%figure(4); clf; hold on;
   %%imagesc(Z); colorbar;
   %%disp(sprintf('min is %g, %g', min(min(Z)), max(max(Z))));
   %
   %S=[];
   %for l=1:length(jump_indices)
   %   for t = 1:length(P1)
   %      if Z(l, t) == max(max(Z))
   %         S = [S, - P1(t)+jump_indices(l)];
   %      end
   %   end
   %end
   %figure(5); clf; hold on;
   %plot(S, 'r*');
   %
   %% Find which shift occurs most often with +/-d variation.
   %Q=0*S;
   %for i=1:length(S)
   %   Q(i) = length(find( S >= S(i)-d & S<= S(i)+ d));
   %end
   %% Out of all very similar shifts, choose the one occuring most often
   %T=S(find(Q==max(Q)));
   %R=0*T;
   %for i=1:length(T)
   %   R(i) = length(find( T(i) == T));
   %end
   %T1 = T(find(R==max(R)));
   %SHIFT=T1(1);
   %disp(sprintf('Shift is %d %d', d, SHIFT));
   %
   %P1_shift = P1 + SHIFT;
   %
   %figure(2); clf; hold on;
   %plot(I0, mean_disparity', 'b');
   %plot(I1+SHIFT, -dx1, 'r');
   %plot(jump_indices, mean_disparity(jump_indices), 'b*');
   %plot(P1_shift, -dx1(P1), 'r*');
   %
   %Tall=[];
   %V = [];
   %for v = 1:length(jump_indices)
   %   T = find(P1_shift >= jump_indices(v) - d & P1_shift <= jump_indices(v) + d);
   %   if length(T) > 0
   %      V = [V, v];
   %      Tall=[Tall, T];
   %   end
   %end
   %
   %plot(jump_indices(V), mean_disparity(jump_indices(V))+0.1, 'g*')
   %plot(P1_shift(Tall), -dx1(P1(Tall))+0.1, 'g*');
   %
