fig = 1;

if fig == 1
   file='ccdx.txt';
else
   file='ccdy.txt';
end

disp(sprintf('loading: %s', file));
dx0 = load(file);
[m, n] = size(dx0);

sx0=find_ccds_aux(dx0);
I0=1:(length(dx0));
P0 = sx0(1, :);
H0 = sx0(2, :);

period = 708;
shift  = -119;
P = period*(1:n) + shift;
I = find(P <= n);
P = P(I);

% Keep one CCD per period
wid = 200;
I=[];
for i=1:length(P)
   J = find( P0 >= P(i) - wid & P0 <= P(i) + wid);
   if length(J) == 0
      continue
   end
   K = find( abs(H0(J)) == max(abs(H0(J))) );
   I = [I, J(K(1))];
end

figure(fig); clf; hold on;

P0 = P0(I);
H0 = H0(I);
plot(dx0, 'b');
plot(P0, dx0(P0), 'b*');
plot(P0, H0, 'r');
plot(P0, H0, 'r*');

p = mean(diff(P0));
P0 - p*1:length(P0)

