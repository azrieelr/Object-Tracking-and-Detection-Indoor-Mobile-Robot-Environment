classdef (Sealed)BinaryOccupancyGrid < robotics.algs.internal.OccupancyGridBase
    %BINARYOCCUPANCYGRID Create a binary occupancy grid
    %   BINARYOCCUPANCYGRID creates an occupancy grid map. Each cell has
    %   a value representing the occupancy status of that cell. An occupied
    %   location is represented as true (1) and a free location is false (0).
    %
    %   MAP = robotics.BinaryOccupancyGrid(W, H) creates a 2D binary
    %   occupancy grid object representing a world space of width(W) and
    %   height(H) in meters. The default grid resolution is 1 cell per meter.
    %
    %   MAP = robotics.BinaryOccupancyGrid(W, H, RES) creates a BinaryOccupancyGrid
    %   object with resolution(RES) specified in cells per meter.
    %
    %   MAP = robotics.BinaryOccupancyGrid(W, H, RES, 'world') creates a
    %   BinaryOccupancyGrid object and  specifies the map size (W and H)
    %   in the world coordinates. This is also the default value.
    %
    %   MAP = robotics.BinaryOccupancyGrid(M, N, RES, 'grid') returns a
    %   BinaryOccupancyGrid object and specifies a grid size of M rows and N columns.
    %   RES specifies the cells per meter resolution.
    %
    %   MAP = robotics.BinaryOccupancyGrid(P) creates a binary
    %   occupancy grid object from the values in the matrix,
    %   P. The size of the grid matches the matrix with each cell value
    %   interpreted from that matrix location. Matrix, P, may contain
    %   any numeric type with zeros(0) and ones(1).
    %
    %   MAP = robotics.BinaryOccupancyGrid(P, RES) creates a BinaryOccupancyGrid
    %   object from matrix, P, with RES specified in cells per meter.
    %
    %   BinaryOccupancyGrid properties:
    %       GridSize            - Size of the grid in [rows, cols] (number of cells)
    %       Resolution          - Grid resolution in cells per meter
    %       XWorldLimits        - Minimum and maximum values of X
    %       YWorldLimits        - Minimum and maximum values of Y
    %       GridLocationInWorld - Location of grid in world coordinates
    %
    %
    %   BinaryOccupancyGrid methods:
    %       setOccupancy    - Set occupancy of a location
    %       getOccupancy    - Get occupancy of a location
    %       grid2world      - Convert grid indices to world coordinates
    %       world2grid      - Convert world coordinates to grid indices
    %       show            - Show grid values in a figure
    %       inflate         - Inflate each occupied grid location
    %       copy            - Create a copy of the object
    %       occupancyMatrix - Convert binary occupancy grid to logical matrix
    %
    %
    %   Example:
    %
    %       % Create a 2m x 2m empty map
    %       map = robotics.BinaryOccupancyGrid(2,2);
    %
    %       % Create a 10m x 10m empty map with resolution 20
    %       map = robotics.BinaryOccupancyGrid(10, 10, 20);
    %
    %       % Create a map from a matrix with resolution 20
    %       p = eye(100);
    %       map = robotics.BinaryOccupancyGrid(p, 20);
    %
    %       % Check occupancy of the world location (0.3, 0.2)
    %       value = getOccupancy(map, [0.3 0.2]);
    %
    %       % Set world position (1.5, 2.1) as occupied
    %       setOccupancy(map, [1.5 2.1], 1);
    %
    %       % Get the grid cell indices for world position (2.5, 2.1)
    %       ij = world2grid(map, [2.5 2.1]);
    %
    %       % Set the grid cell indices to unoccupied
    %       setOccupancy(map, [1 1], 0, 'grid');
    %
    %       % Set the grid cell indices using partial string argument
    %       setOccupancy(map, [1 1], 0, 'g');
    %
    %       % Show binary occupancy grid in Graphics figure
    %       show(map);
    %
    %   See also robotics.OccupancyGrid, robotics.PRM, robotics.PurePursuit.
    
    %   Copyright 2014-2017 The MathWorks, Inc.
    
    %#codegen
    
    properties (Access = {?robotics.algs.internal.GridAccess, ...
            ?robotics.algs.internal.OccupancyGridBase})
        %Grid The occupancy grid data
        %   A matrix that stores the values for grid cells.
        Grid
    end
    
    methods
        function obj = BinaryOccupancyGrid(varargin)
            %BinaryOccupancyGrid Constructor
            
            % Parse input arguments
            narginchk(1,4);
            [resolution, isGrid, mat, width, height, isMat]...
                = obj.parseConstructorInputs(varargin{:});
            
            obj.Resolution = resolution;
            
            % Construct grid from a matrix input
            if isMat
                obj.Grid = mat;
                obj.GridSize = size(obj.Grid);
                return;
            end
            
            % Construct empty grid from width and height
            if isGrid
                obj.Grid = mat;
            else
                gridsize = size(mat);
                % Throw a warning if we round off the grid size
                if any(gridsize ~= ([height, width]*obj.Resolution))
                    coder.internal.warning(...
                        'robotics:robotalgs:occgridcommon:RoundoffWarning');
                end
                
                obj.Grid = mat;
            end
            obj.GridSize = size(obj.Grid);
        end
        
        function cpObj = copy(obj)
            %copy Creates a copy of the object
            %   cpObj = copy(obj) creates a deep copy of the Binary
            %   Occupancy Grid object with the same properties.
            %
            %   Example:
            %       % Create a binary occupancy grid of 10m x 10m world representation
            %       map = robotics.BinaryOccupancyGrid(10, 10);
            %
            %       % Create a copy of the object
            %       cpObj = copy(map);
            %
            %       % Access the class methods from the new object
            %       setOccupancy(cpObj,[2 4],true);
            %
            %       % Delete the handle object
            %       delete(cpObj)
            
            if isempty(obj)
                cpObj = robotics.BinaryOccupancyGrid.empty(0,1);
            else
                % Create a new object with the same properties
                cpObj = robotics.BinaryOccupancyGrid(obj.GridSize(1),...
                    obj.GridSize(2),obj.Resolution,'grid');
                
                % Assign the grid data to the new object handle
                cpObj.Grid = obj.Grid;
                cpObj.GridLocationInWorld = obj.GridLocationInWorld;
            end
        end
        
        function value = getOccupancy(obj, pos, frame)
            %getOccupancy Get occupancy value for one or more positions
            %   VAL = getOccupancy(MAP, XY) returns an N-by-1 array of
            %   occupancy values for N-by-2 array, XY. Each row of the
            %   array XY corresponds to a point with [X Y] world coordinates.
            %
            %   VAL = getOccupancy(MAP, XY, 'world') returns an N-by-1 array of
            %   occupancy values for N-by-2 array XY in world coordinates.
            %   This is the default value.
            %
            %   VAL = getOccupancy(MAP, IJ, 'grid') returns an N-by-1
            %   array of occupancy values for N-by-2 array IJ. Each row of
            %   the array IJ refers to a grid cell index [X,Y].
            %
            %   Example:
            %       % Create a binary occupancy grid and get occupancy
            %       % values for a position
            %       map = robotics.BinaryOccupancyGrid(10, 10);
            %
            %       % Get occupancy of the world coordinate (0, 0)
            %       value = getOccupancy(map, [0 0]);
            %
            %       % Get occupancy of multiple coordinates
            %       [X, Y] = meshgrid(0:0.5:5);
            %       values = getOccupancy(map, [X(:) Y(:)]);
            %
            %       % Get occupancy of the grid cell (1, 1)
            %       value = getOccupancy(map, [1 1], 'grid');
            %
            %       % Get occupancy of multiple grid cells
            %       [I, J] = meshgrid(1:5);
            %       values = getOccupancy(map, [I(:) J(:)], 'grid');
            %
            %   See also robotics.BinaryOccupancyGrid, setOccupancy
            
            
            isGrid = false;
            % If optional argument present then parse it separately
            if nargin > 2
                isGrid = obj.parseOptionalFrameInput(frame, 'getOccupancy');
            end
            
            % Validate position or subscripts and convert it to indices
            indices = obj.getIndices(pos, isGrid, 'getOccupancy');
            value = obj.Grid(indices);
        end
        
        function setOccupancy(obj, pos, value, frame)
            %setOccupancy Set occupancy value for one or more positions
            %   setOccupancy(MAP, XY, VAL) assigns the scalar occupancy
            %   value, VAL to each coordinate specified in the N-by-2 array,
            %   XY. Each row of the array XY corresponds to a point with
            %   [X Y] world coordinates.
            %
            %   setOccupancy(MAP, XY, VAL) assigns each element of the
            %   N-by-1 vector, VAL to the coordinate position of the
            %   corresponding row of the N-by-2 array, XY.
            %
            %   setOccupancy(MAP, XY, VAL, 'world') specifies the N-by-2
            %   array XY as world coordinates. This is also the default
            %   value.
            %
            %   setOccupancy(MAP, IJ, VAL, 'grid') assigns occupancy values
            %   to the grid positions specified by each row of the N-by-2
            %   array, IJ, which refers to the [row, col] index from each row
            %   in the array.
            %
            %   Example:
            %       % Create a binary occupancy grid and set occupancy
            %       % values for a position
            %       map = robotics.BinaryOccupancyGrid(10, 10);
            %
            %       % Set occupancy of the world coordinate (0, 0)
            %       setOccupancy(map, [0 0], 1);
            %
            %       % Set occupancy of multiple coordinates
            %       [X, Y] = meshgrid(0:0.5:5);
            %       values = ones(numel(X),1);
            %       setOccupancy(map, [X(:) Y(:)], values);
            %
            %       % Set occupancy of the grid cell (1, 1)
            %       setOccupancy(map, [1 1], 1, 'grid');
            %
            %       % Set occupancy of multiple grid cells
            %       [I, J] = meshgrid(1:5);
            %       setOccupancy(map, [I(:) J(:)], 1, 'grid');
            %
            %       % Set occupancy of multiple grid cells
            %       [I, J] = meshgrid(1:5);
            %       values = ones(numel(I),1);
            %       setOccupancy(map, [I(:) J(:)], values, 'grid');
            %
            %   See also robotics.BinaryOccupancyGrid, getOccupancy
            
            
            isGrid = false;
            % If optional argument present then parse it separately
            if nargin > 3
                isGrid = obj.parseOptionalFrameInput(frame, 'setOccupancy');
            end
            
            % Validate values
            value = obj.validateOccupancyValues(value, size(pos, 1), 'VAL');
            
            % Validate position or subscripts and convert it to indices
            index = obj.getIndices(pos, isGrid, 'setOccupancy');
            obj.Grid(index) = value(:);
        end
        
        function idx = world2grid(obj, pos)
            %WORLD2GRID Convert world coordinates to grid indices
            %   IJ = WORLD2GRID(MAP, XY) converts an N-by-2 array of world
            %   coordinates, XY, to an N-by-2 array of grid indices, IJ. The
            %   input, XY, is in [X Y] format. The output grid indices, IJ,
            %   are in [ROW COL] format.
            %
            %   Example:
            %       % Create a binary occupancy grid and convert world
            %       % coordinates to grid indices
            %       % Create a 10m x 10m world representation
            %       map = robotics.BinaryOccupancyGrid(10, 10);
            %
            %       % Get grid indices from world coordinates
            %       ij = world2grid(map, [0 0])
            %
            %       % Get grid indices from world coordinates
            %       [x y] = meshgrid(0:0.5:2);
            %       ij = world2grid(map, [x(:) y(:)])
            
            pos = obj.validatePosition(pos, obj.XWorldLimits, obj.YWorldLimits, 'world2grid', 'XY');
            
            % Convert world coordinate to grid indices
            idx = worldToGridPrivate(obj, pos);
        end
        
        function pos = grid2world(obj, idx)
            %GRID2WORLD Convert grid indices to world coordinates
            %   XY = GRID2WORLD(MAP, IJ) converts an N-by-2 array of grid
            %   indices, IJ, to an N-by-2 array of world coordinates, XY. The
            %   input grid indices, IJ, are in [ROW COL] format. The output,
            %   XY, is in [X Y] format.
            %
            %   Example:
            %       % Create a binary occupancy grid and convert grid
            %       % indices to world coordinates
            %       % Create a 10m x 10m world representation
            %       map = robotics.BinaryOccupancyGrid(10, 10);
            %
            %       % Get world coordinates from grid indices
            %       xy = grid2world(map, [1 1])
            %
            %       % Get world coordinates from grid indices
            %       [i j] = meshgrid(1:5);
            %       xy = world2grid(map, [i(:) j(:)])
            
            idx = obj.validateGridIndices(idx, obj.GridSize, 'grid2world', 'IJ');
            
            % Convert grid index to world coordinate
            pos = gridToWorldPrivate(obj, idx);
        end
        
        function inflate(obj, varargin)
            %INFLATE Inflate the occupied positions by a given amount
            %   INFLATE(MAP, R) inflates each occupied position of the binary
            %   occupancy grid by at least R meters. Each cell of the binary
            %   occupancy grid is inflated by number of cells which is the
            %   closest integer higher than the value MAP.Resolution*R.
            %
            %   INFLATE(MAP, R, 'grid') inflates each cell of the binary
            %   occupancy grid by R cells.
            %
            %   Note that the inflate function does not inflate the
            %   positions past the limits of the grid.
            %
            %   Example:
            %       % Create a binary occupancy grid and inflate map
            %       bmat = eye(100);
            %       map = robotics.BinaryOccupancyGrid(bmat);
            %
            %       % Create a copy of the map for inflation
            %       cpMap = copy(map);
            %
            %       % Inflate occupied cells using inflation radius in
            %       % meters
            %       inflate(cpMap, 0.1);
            %
            %       % Inflate occupied cells using inflation radius in
            %       % number of cells
            %       inflate(cpMap, 2, 'grid');
            %
            %   See also robotics.BinaryOccupancyGrid, copy
            
            narginchk(2,3);
            obj.Grid = obj.inflateGrid(obj.Grid, varargin{:});
        end
        
        function imageHandle = show(obj, varargin)
            %SHOW Display the binary occupancy grid in a figure
            %   SHOW(MAP) displays the MAP binary occupancy grid in the
            %   current axes with the axes labels representing the world
            %   coordinates.
            %
            %   SHOW(MAP, 'world') displays the MAP with the axes labels
            %   representing the world coordinates of the MAP. This is the
            %   default.
            %
            %   SHOW(MAP, 'grid') displays the MAP binary occupancy grid in
            %   the current axes with the axes of the figure representing
            %   the grid indices.
            %
            %   HIMAGE = SHOW(MAP, ___) returns the handle to the image
            %   object created by show.
            %
            %   SHOW(MAP,___,Name,Value) provides additional options specified
            %   by one or more Name,Value pair arguments. Name must appear
            %   inside single quotes (''). You can specify several name-value
            %   pair arguments in any order as Name1,Value1,...,NameN,ValueN:
            %
            %       'Parent'        - Handle of an axes that specifies
            %                         the parent of the image object
            %                         created by show.
            %
            %   Example:
            %       % Create a binary occupancy grid and display
            %       map = robotics.BinaryOccupancyGrid(eye(5));
            %
            %       % Display the occupancy with axes showing the world
            %       % coordinates
            %       gh = show(map);
            %
            %       % Display the occupancy with axes showing the grid
            %       % indices
            %       gh = show(map, 'grid');
            %
            %       % Display the occupancy with axes showing the world
            %       % coordinates and specify a parent axes
            %       fh = figure;
            %       ah = axes('Parent', fh);
            %       gh = show(map, 'world', 'Parent', ah);
            %
            %   See also robotics.BinaryOccupancyGrid
            
            [axHandle, isGrid] = obj.showInputParser(varargin{:});
            [axHandle, imghandle] = showGrid(obj, obj.Grid, axHandle, isGrid);
            title(axHandle, ...
                message('robotics:robotalgs:binaryoccgrid:FigureTitle').getString);
            
            % Only return handle if user requested it.
            if nargout > 0
                imageHandle = imghandle;
            end
        end
        
        function mat = occupancyMatrix(obj)
            %OCCUPANCYMATRIX Export binary occupancy grid as a matrix
            %   MAT = OCCUPANCYMATRIX(MAP) returns occupancy values stored in the
            %   binary occupancy grid object as a logical matrix of size
            %   GridSize.
            %
            %   Example:
            %       % Create an occupancy grid
            %       map = robotics.BinaryOccupancyGrid(eye(10));
            %
            %       % Export occupancy grid as a matrix
            %       mat = occupancyMatrix(map);
            %
            %   See also robotics.OccupancyGrid, getOccupancy
            
            mat = obj.Grid;
        end
    end
    methods (Access = protected)
        function  mat = processMatrix(~, varargin)
            %processMatrix Process construction input and create grid
            % Supports two input patterns:
            %   processMatrix(obj, rows,columns);
            %   processMatrix(obj, mat);
            if nargin == 3
                rows = varargin{1};
                columns = varargin{2};
                % To ensure varsize rows/columns are converted into a scalar
                rows = rows(1);
                columns = columns(1);
                mat = false(rows, columns);
            elseif nargin == 2
                mat = logical(varargin{1});
            end
        end
    end
    %================================================================
    methods (Static, Access = {?robotics.algs.internal.OccupancyGridBase})
        function validateMatrixInput(P)
            %validateMatrixInput Validates the matrix input
            validateattributes(P, {'numeric','logical'}, ...
                {'2d', 'real', 'nonempty','nonnan'},'BinaryOccupancyGrid', 'P', 1);
        end
        
        function className = getClassName()
            className = 'BinaryOccupancyGrid';
        end
        
        function values = validateOccupancyValues(values, len, name)
            %validateOccupancyValues Validate occupancy value vector
            
            validateattributes(values, {'logical','numeric'}, ...
                {'real', 'vector', 'nonnan', 'nonempty'}, 'setOccupancy', name);
            
            lengthValues =  length(values) ~= 1 && length(values) ~= len;
            coder.internal.errorIf(lengthValues,...
                'robotics:robotalgs:occgridcommon:InputSizeMismatch');
            
            values = logical(values);
        end
    end
end