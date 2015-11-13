function b = find_moving_avg(a)

   wid = 700;
   wid2 = floor(wid/2);
   len = length(a);
   b = a + NaN;

   ss = 1; % first nonnan
   for i=1:len
      if ~isnan(a(i))
         ss = i;
         break;
      end
   end
   
   ee = len; % last nonnan
   for i=len:-1:1
      if ~isnan(a(i))
         ee = i;
         break;
      end
   end

   for i=1:len

      % average using a window of size wid
      
      s0 = max(i-wid2, ss);
      e0 = min(i+wid2, ee);

      if s0 < i & e0 > i
         s0 = max(s0, i - (e0-i));
         e0 = min(e0, i + (i-s0));
         b(i) = mean(a(s0:e0));
      else
         b(i) = a(i);
      end
      
%         p = i-k;
%         q = i+k;
%
%         if (p >= 1) & (~isnan(a(p)))
%            sum = sum + a(p);
%            num = num + 1;
%         end
%
%         if num >= wid
%            break;
%         end
%
%         if (q ~= p) & (q <= len) & (~isnan(a(q)))
%            sum = sum + a(q);
%            num = num + 1;
%         end
%
%         if num >= wid
%            break;
%         end
%
%      end
%
%      if num >= 1
%         b(i) = sum/num;
%      end
      
   end

   
