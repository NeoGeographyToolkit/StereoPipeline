function main(do_find, dirs, pitches, plotid)

   % if do_find is 1, we actually find and save the CCDs,
   % otherwise we just examine the current averaged disparties.

   if nargin < 4 % Did the user pass in the pitch list?
      error('Missing arguments!')
   end

   if length(pitches) ~= length(dirs)
      error('Must have one pitch per directory')
   end
   %if length(col_starts) ~= length(dirs)
   %   error('Must have col start per directory')
   %end
   
   % Generate list of dx and dy file paths
   dx_file_paths={};
   dy_file_paths={};
   for i=1:length(dirs)
      dir = dirs{i};
      dx_path=[dir '/dx.txt'];
      dy_path=[dir '/dy.txt'];
      dx_file_paths = [dx_file_paths, dx_path];
      dy_file_paths = [dy_file_paths, dy_path];
   end
   
   % Seperately handle dx then dy
   do_plot(do_find, dx_file_paths, pitches, plotid);
   title('x');
   
   do_plot(do_find, dy_file_paths, pitches, 1+plotid); 
   title('y');
end

%--------------------------------------------------------

function disparity_data = scale_by_pitch(disparity_data, pitch)
   % Compensate for the fact that ccd artifacts are spaced closer for larger pitch
   % - Do this by interpolating a new vector at the default pitch size.
   % - The larger pitch size effectively represents a downsampled input image.
   BASE_PITCH = 8.0000000000e-03; % The default pitch value
   I = (BASE_PITCH/pitch)*(1:(10*length(disparity_data))); % Get normal pitch pixel locations to a large distance
   J=find(I > length(disparity_data)); % Find all those locations that fall out of bounds
   I = I( 1:(J(1)-1)-1 ); % Crop to only in-bounds locations
   disparity_data = interp1(1:length(disparity_data), disparity_data', I, 'linear')';
end

% The main working function!
function do_plot(do_find, disparity_file_paths, pitches, fig)
   
   PRINT_EACH_FILE = true;
   
   % TODO: Should input values outside +1/-1 be replaced with NaN?

   % TODO: How to deal with files of different sizes?
   %--> Very unlikely that the pixels actually line up with eachother!
   %--> Would like to be able to crop out the borders of some images and still have them line up.
   %--> To do this need a column offset for each input file.

   % Loop through each of the input disparity files and load the data into X.
   X = [];
   for i=1:length(disparity_file_paths)
      disparity_path=disparity_file_paths{i};
      if exist(disparity_path, 'file') == 2  % If the file exists...
         %disp(sprintf('Loading %s', disparity_path)); 
      else
         error(sprintf('Missing file: %s', disparity_path));
      end
      % Load the data and extract the crop amount
      disparity_data = load(disparity_path); 
      col_start      = disparity_data(1); % Data outside this range is junk, existing zero handling should
      col_stop       = disparity_data(2); % take care of it.
      %disparity_data = disparity_data(3:end);

      % Scale the data by the pitch
      % - This adjusts all input vectors to the same size pixels, resizing the arrays.
      disparity_data = scale_by_pitch(disparity_data, pitches(i));
      
      disp(['Disparity length = ', num2str(length(disparity_data))])
      
      [existing_data_length, n_files_stored] = size(X);
      % Deal with size mis-matches by either growing X or disparity_data.
      % TODO: Our code does not handle this case at all!
      if existing_data_length > 0 % If this is not the first file loaded
         new_data_length = length(disparity_data);
         
         %if new_data_length ~= existing_data_length
         %   error(sprintf('Input data lengths %d and %d do not match!', new_data_length, existing_data_length));
         %end
         % Try not to use this old code!         
         if new_data_length < existing_data_length % Pad out the new data
            disparity_data = [disparity_data' zeros(1, existing_data_length - new_data_length)]';
         elseif existing_data_length < new_data_length % Pad out the existing data
            Y = zeros(new_data_length, n_files_stored);
            Y(1:existing_data_length, 1:n_files_stored) = X;
            X = Y;
            [data_length, n_files_stored] = size(X); % Update these values
         end
         
      end % End case handling existing data

      % Now that the sizes are equalized, append the new data.
      X = [X disparity_data];
      
   end % End loop through disparity files
   
   % Transpose the data so that the first dimension is smaller
   X = X';
   [n_files_stored, data_length] = size(X);
   %disp(sprintf('size of X is %d %d', n_files_stored, existing_data_length));
   
   % Set to NaN all values within a buffer of a zero value near the ends of
   %  each loaded data set.
   edge_search_dist = round(data_length/3);
   for r=1:n_files_stored

      % Stay away from boundary.
      %BDVAL = 300; % Must tweak this!!!
      BDVAL = 100; % Smaller for working with cropped regions
      
      % Search through the data to find the last? zero value
      col_start = 1;
      for c=1:edge_search_dist % Loop through first third of data
         if X(r, c) == 0
            col_start = max(col_start, c);
         end
      end

      % Do the same thing but coming in from the back
      col_end = data_length;
      for c=(data_length-edge_search_dist):data_length
         if X(r, c) == 0
            col_end = min(col_end, c);
         end
      end
      
      % Move in even farther
      col_start = col_start + BDVAL;
      col_end   = col_end - BDVAL;
      
      % Clear all the values outside the boundaries
      for c=1:col_start
         X(r, c) = NaN;
      end
      for c=col_end:data_length
         X(r, c) = NaN;
      end
      
   end % End loop through stored files
   
   wid = 35;

   % These colors are rotated through as files are plotted
   colors  = {'b', 'r', 'g', 'c', 'k'};
   brushes = {'-', '--', '-.', ':', '-', '--', '-.', ':', '-', '--', '-.', ':', '-', '--', '-.', ':'};

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
         color_index = rem(r-1, length(colors))+1;
         brush_index = floor((r-1) / length(colors))+1;
         line_color = [brushes{brush_index}, colors{color_index}];
         vertical_offset = sep_size*(r+1); % Visually seperate the plots
         plot(Y + vertical_offset, line_color);
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
   
   % Set the plot so that the axes have the right numbers when moving
   h1 = zoom;
   h2 = pan;
   set(gca,'XTickLabelMode','auto')
   set(gca,'XTickLabel',num2str(get(gca,'XTick').'))
   set(h1,'ActionPostCallback',@mypostcallbackX);
   set(h2,'ActionPostCallback',@mypostcallbackX);

   if do_find ~= 0
      find_ccds_aux(mean_X, fig)
   end
end

% Callback function to keep axes updated when zooming
function mypostcallbackX(obj,evd)
   set(gca,'XTickLabelMode','auto')
   set(gca,'XTickLabel',num2str(get(gca,'XTick').'))
end

% Split up a string based on spaces
function b = split(a)
   b = strread(a,'%s','delimiter',' ')
end
