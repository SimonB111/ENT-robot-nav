## drillViz/drilling.py

Command-line tool to generate and visualize drilling trajectories on anatomical meshes using interactive point selection and spline-based path planning.

### Basic usage

python3 drilling.py <mesh_input>

- `mesh_input` (required): Path to the input mesh file (e.g. STL)

### Options

- `--meters2mm`: Convert mesh units from meters to millimeters (scale factor of 1000).

**Example:**

python3 drilling.py path/to/anatomy.stl --meters2mm

### Features

- Interactive 3D mesh visualization with PyVista
- Right-click to select 3 target points (start, midpoint, end) on the mesh surface
- Automatic drilling path generation through surface intersection
- Smooth spline interpolation for trajectory smoothing
- Real-time drill animation with proper orientation (45-degree drilling angle)
- Press 'c' to reset and select new target points
- Visualizes plane intersection, spline path, and target points in 3D
