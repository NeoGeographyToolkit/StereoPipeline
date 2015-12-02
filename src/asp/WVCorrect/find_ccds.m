function main(do_find, dirs, pitches, plotid)

   % if do_find is 1, we actually find and save the CCDs,
   % otherwise we just examine the current averaged disparties.

   if nargin == 2 % Did the user pass in the pitch list?
      pitches = [];
   end
   
   % Generate list of dx and dy file paths
   ax={};
   ay={};
   for i=1:length(dirs)
      dir = dirs{i};
      ax0=[dir '/dx.txt'];
      ay0=[dir '/dy.txt'];
      ax = [ax, ax0];
      ay = [ay, ay0];
   end
   
   % Seperately handle dx then dy
   do_plot(do_find, ax, pitches, plotid);
   title('x');
   
   %do_plot(do_find, ay, pitches, 1+plotid); 
   %title('y');

%--------------------------------------------------------

function disparity_data = scale_by_pitch(disparity_data, pitch)
   % compensate for the fact that ccd artifacts are spaced closer
   % for larger pitch
   BASE_PITCH = 8.0000000000e-03; % The default pitch value
   I = (BASE_PITCH/pitch)*(1:(10*length(disparity_data)));
   J=find(I > length(disparity_data));
   I = I(1:(J(1)-1));
   disparity_data = interp1(1:length(disparity_data), disparity_data', I, 'linear')';

% The main working function!
function do_plot(do_find, disparity_file_paths, pitches, fig)
   
   PRINT_EACH_FILE = false;
   
   % TODO: Should input values outside +1/-1 be replaced with NaN?

   % Loop through each of the input disparity files and load the data into X.
   X = [];
   for i=1:length(disparity_file_paths)
      disparity_path=disparity_file_paths{i};
      if exist(disparity_path, 'file') == 2  % If the file exists...
         %disp(sprintf('Loading %s', disparity_path)); 
      else
         disp(sprintf('Missing file: %s', disparity_path));
         continue;
      end
      disparity_data = load(disparity_path); 

      if length(pitches) >= i % If pitch value is available, scale the data
         disparity_data = scale_by_pitch(disparity_data, pitches(i));
      end
      
      [existing_data_length, n_files_stored] = size(X);
      % Deal with size mis-matches by either growing X or disparity_data.
      if existing_data_length > 0 % If this is not the first file loaded
         new_data_length = length(disparity_data);
         if new_data_length < existing_data_length % Pad out the new data
            disparity_data = [disparity_data' zeros(1, existing_data_length - new_data_length)]';
         elseif existing_data_length < new_data_length % Pad out the existing data
            Y = zeros(new_data_length, n_files_stored);
            Y(1:existing_data_length, 1:n_files_stored) = X;
            X = Y;
            [data_length, n_files_stored] = size(X); % Update these values
         end
      end

      % Now that the sizes are equalized, append the new data.
      X = [X disparity_data];
      
   end % End loop through disparity files
   
   % Transpose the data so that the first dimension is smaller
   X = X';
   [n_files_stored, data_length] = size(X);
   %disp(sprintf('size of X is %d %d', n_files_stored, existing_data_length));
   
   % Set to NaN all values within a buffer of a zero value near the ends of
   %  each loaded data set.
   for r=1:n_files_stored

      % Stay away from boundary.
      %BDVAL = 300; % Must tweak this!!!
      BDVAL = 100; % Smaller for working with cropped regions
      
      % Search through the data to find the last? zero value
      c0 = round(data_length/3);
      c1 = data_length - c0;
      cs = 1;
      for c=1:c0 % Loop through first third of data
         if X(r, c) == 0
            cs = max(cs, c);
         end
      end

      % Do the same thing but coming in from the back
      ce = data_length;
      for c=(data_length-c0):data_length
         if X(r, c) == 0
            ce = min(ce, c);
         end
      end
      
      % Move in even farther
      cs = cs + BDVAL;
      ce = ce - BDVAL;
      
      % Clear all the values outside the boundaries
      for c=1:cs
         X(r, c) = NaN;
      end
      for c=ce:data_length
         X(r, c) = NaN;
      end
      
   end % End loop through stored files
   
   wid = 35;

   % These colors are rotated through as files are plotted
   colors=['b', 'r', 'g', 'c', 'k', 'b', 'r', 'g', 'c', 'b', 'r', 'g', 'k', 'c'];

   figure(fig); clf; hold on;
   sep_size=0.0;
   if do_find
      sep_size = 0.25; % vertical gap between individual curves, for visibility
   end
   
   
   for r=1:n_files_stored
      if r <= length(disparity_file_paths) 
         disp(sprintf('doing %s', disparity_file_paths{r}));
      end

      % Subtract out a smoothed version of this file's data
      Y = X(r, :) - find_moving_avg(X(r, :));
      X(r, :) = Y;
      
      if (PRINT_EACH_FILE) % Plot the data set, vertically shifted
         % Select a color for this file and plot the shifted data
         r2 = rem(r-1, length(colors))+1;
         vertical_offset = sep_size*(r+1); % Visually seperate the plots
         plot(Y + vertical_offset, colors(r2));
      end
   end % End loop through stored files


   % Take the mean of all of the input data
   mean_X = zeros(1, data_length);
   for c=1:data_length
      sum = 0;
      num = 0;
      for r=1:n_files_stored
         if ~isnan(X(r, c))
            sum = sum + X(r, c);
            num = num + 1;
         end
      end
      if num > 0
         mean_X(1, c) = sum/num;
      end
   end % End loop through data

   mean_line_width = 1;
   if PRINT_EACH_FILE
      mean_line_width = 3;
   end
   plot(mean_X, 'm', 'LineWidth', mean_line_width);
   %ylim([-1, 1])
   

   h1 = zoom;
   h2 = pan;
   set(gca,'XTickLabelMode','auto')
   set(gca,'XTickLabel',num2str(get(gca,'XTick').'))
   set(h1,'ActionPostCallback',@mypostcallbackX);
   set(h2,'ActionPostCallback',@mypostcallbackX);

   if do_find ~= 0
      find_ccds_aux(mean_X, fig)
   end


function mypostcallbackX(obj,evd)
   % Redo conversion to decimal format
   set(gca,'XTickLabelMode','auto')
   set(gca,'XTickLabel',num2str(get(gca,'XTick').'))


% Split up a string based on spaces
function b = split(a)
   b = strread(a,'%s','delimiter',' ')

