function find_ccds_aux(dx0, fig)

   n = length(dx0);
   
   sx0=find_ccds_aux2(dx0);

   I0=1:(length(dx0));
   P0 = sx0(1, :);
   H0 = sx0(2, :);
   
   wid = 60;
   period = 705;
   shift  = -80;
   [P0, H0] = sparse_ccds(wid, period, shift, n, P0, H0);

   figure(fig); hold on;
   plot(dx0, 'r');
   plot(P0, dx0(P0), 'b*');
   plot(P0, H0, 'b');

   %[period, shift] = find_true_period(P0, H0);
   
%   if fig == 1
%      figure(fig+10);
%      clf; hold on;
%      plot(diff(P0), 'b');
%      plot(1000*H0, 'r');
%   end
   
   format long g;
   T = [P0' H0']';
   if fig == 1
      file='ccdx.txt';
      file2='avgx.txt';
   else
      file='ccdy.txt';
      file2='avgy.txt';
   end

   dx0 = dx0';
   disp(sprintf('saving %s', file));
   disp(sprintf('saving %s', file2));
   save(file, '-ascii', '-double', 'T');
   save(file2, '-ascii', '-double', 'dx0');

function [period, shift] = find_true_period(P0, H0)
   % The true period is associated with the hugest hump
   n = length(P0);
   Q = zeros(n-1, 1);
   for i=1:(n-1)
      Q(i) = abs(H0(i)) + abs(H0(i+1));
   end
   I = find(Q == max(Q));
   i = I(1);
   period = P0(i+1) - P0(i);
   shift = P0(i) - i*period;
   while shift > 0
      shift = shift - period;
   end

   plot(P0(i), H0(i), 'g*');
   disp(sprintf('--period and shift %g %g', period, shift));
   
   
function [P0, H0] = sparse_ccds(wid, period, shift, n, P0, H0)
   
   % Keep one CCD per period
   P = period*(1:n) + shift;
   I = find(P <= n);
   P = P(I);
   I=[];
   for i=1:length(P)
      J = find( P0 >= P(i) - wid & P0 <= P(i) + wid);
      if length(J) == 0
         continue
      end
      K = find( abs(H0(J)) == max(abs(H0(J))) );
      I = [I, J(K(1))];
   end
   P0 = P0(I);
   H0 = H0(I);
   
function U = find_ccds_aux2(B)
   

   do_plot = 0; figno=0;
   SHIFT=0;
   if do_plot
      figure(figno); clf; hold on;
   end
   
   %B = find_avg(A, col);
   %Bt=B';
   %save('Bm.txt', '-ascii', '-double', 'Bt');
   %return
   
   %B0 = find_avg(A0, col);
   
   wid = 35;
   
   Q = B*0 + NaN;
   for i=1:length(B)
      %if i-wid >= 1 & i+wid<= length(S)
      if i-wid >= 1 & i+2*wid <= length(B)
         % Document this!!!
         % To do: Improve the accuracy by doing slopes with least squares
         Q(i) = 1.5*(B(i+wid) - B(i)) - 0.5*(B(i + 2*wid) - B(i-wid));
         %Q(i) = wid*(2*S(i) - S(i-wid) - S(i + wid))/2; 
      end
   end
   
   % To do: Use some other criterion
   cutoff = 0.01; % temporary!!! % CCD artifacts must be less than this
   max_cutoff = 1.5; % no ccd artifacts more than that are expected
   wid2 = ceil(1.5*wid); % leave only largest maxium in length wid2 on each side
   
   G = [];
   V = [];
   for i=1:length(Q)
      if isnan(Q(i)) 
         continue
      end
      
      if abs(Q(i)) < cutoff | abs(Q(i)) > max_cutoff
         continue
      end
      
      if i-wid2 < 1 | i + wid2 > length(Q)
         continue
      end
      
      % To do: This needs more thinking, can result in a chain
      % reaction eliminating too many artifacts.
      is_good = 1;
      for l = (i-wid2):(i+wid2)
         if abs(Q(i)) < abs(Q(l))
            is_good = 0;
         end
      end
      
      if is_good
         G = [G, i];
         V = [V, Q(i)];
      end
   end
   
   w2 = floor(wid/2);
   %G = G + w2; % to be at center of CCD defect region
   format long g
   %[G'+w2 V']'
   
   U = [G'+w2+SHIFT V']';
   
   
   %return
   %
   %%
   %%plot(sx0(1, K0), 0*dx0(sx0(1, K0)), 'b*')
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
   %%plot(sx0(1, K0), 0*dx0(sx0(1, K0)), 'b*')
   %%plot(sx1(1, K1)+s, 0*dx1(sx1(1, K1)), 'r*')
   %
   %P0 = sx0(1, :);
   %P1 = sx1(1, :);
   %Z = zeros(length(P0), length(P1));
   %for l=1:length(P0)
   %   %disp(sprintf('l=%d', l));
   %   for t = 1:length(P1)
   %      P1_shift = P1 - P1(t)+P0(l);
   %      for v = 1:length(P0)
   %         T = find(P1_shift >= P0(v) - d & P1_shift <= P0(v) + d);
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
   %for l=1:length(P0)
   %   for t = 1:length(P1)
   %      if Z(l, t) == max(max(Z))
   %         S = [S, - P1(t)+P0(l)];
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
   %plot(I0, dx0', 'b');
   %plot(I1+SHIFT, -dx1, 'r');
   %plot(P0, dx0(P0), 'b*');
   %plot(P1_shift, -dx1(P1), 'r*');
   %
   %Tall=[];
   %V = [];
   %for v = 1:length(P0)
   %   T = find(P1_shift >= P0(v) - d & P1_shift <= P0(v) + d);
   %   if length(T) > 0
   %      V = [V, v];
   %      Tall=[Tall, T];
   %   end
   %end
   %
   %plot(P0(V), dx0(P0(V))+0.1, 'g*')
   %plot(P1_shift(Tall), -dx1(P1(Tall))+0.1, 'g*');
   %
