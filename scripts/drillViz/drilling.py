#!/usr/bin/env python3
# Purpose: Generate and animate bone drilling trajectory on a mesh based on user-selected target points


import pyvista as pv
from scipy import interpolate
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as Rot
import time
import numpy as np
import argparse


class trajectoryGenerator:
    """
    Class to generate and visualize drilling trajectory on a mesh based on user-selected target points.
    """
    def __init__(self, meshInput, scale=1.0):
        """Initialize the trajectory generator with the input mesh and scaling factor.
        Args:
            meshInput (str): Path to the input mesh file.
            scale (float, optional): Scaling factor for the mesh units. Defaults to 1.0.
        """
        self.mesh = pv.read(meshInput)
        self.mesh.points *= scale
        self.mesh.compute_normals(cell_normals=True, point_normals=True, inplace=True)

        self.plotter = pv.Plotter()
        self.plotter.enable_surface_point_picking(callback=self.clickCallback, show_message=False)
        self.plotter.add_mesh(self.mesh, color='lightgray', opacity=1, name='mainMesh')

        drillMesh = pv.read('../example/drill.stl')
        drillMesh.points *= (scale/1.35)
        self.drillActor = self.plotter.add_mesh(
            drillMesh, 
            color='gray', 
            name='drillMesh', 
            pickable=False
        ) # add drill mesh for visualization, not used for path planning

        self.plotter.add_key_event('c', self.resetVisual)

        self.targetPoints = np.zeros((3, 3)) # initialize with 3 points, will be updated with actual target points
        self.numPts = 0

        self.maxGap = 1.0 # max gap between points in mm
        self.endpointTolerance = 0.2 # tolerance for reaching endpoint in mm

        message = "Right click to select 3 target points: start, mid, end. Press 'c' to reset."
        self.plotter.add_text(
                message, 
                position='upper_left', 
                name="colorKey", 
                font_size=10, 
                color='black',
        )

    def getSurfaceNormal(self, point) -> np.ndarray:
        """
        Get the surface normal at a specific point on the mesh by finding the 
        closest face and averaging its vertex normals.
        """
        # don't assume point is on surface, get closest face on mesh
        faceId = self.mesh.find_closest_cell(point)
        
        faceVertexIndices = self.mesh.get_cell(faceId).point_ids
        vNormals = self.mesh.point_data['Normals'][faceVertexIndices]
        avgNormal = np.mean(vNormals, axis=0)
        return avgNormal / np.linalg.norm(avgNormal) # normalize
    
    def getBarycentricNormal(self, query_point) -> np.ndarray:
        """
        Interpolates a normal at a specific point on a 
        PyVista face using barycentric coordinates for smoothness.
        """
        face_id = self.mesh.find_closest_cell(query_point)
        # get the 3 vertex IDs for the face
        v_ids = self.mesh.get_cell(face_id).point_ids
        
        # get the coordinates of the corners (A, B, C)
        a, b, c = self.mesh.points[v_ids]
        
        # get the vertex normals at those corners
        na, nb, nc = self.mesh.point_data['Normals'][v_ids]

        # calculate Barycentric Coordinates (u, v, w)
        # Vectors from corner A
        v0 = b - a
        v1 = c - a
        v2 = query_point - a
        
        # dot products for the Cramer's rule solution
        d00 = np.dot(v0, v0)
        d01 = np.dot(v0, v1)
        d11 = np.dot(v1, v1)
        d20 = np.dot(v2, v0)
        d21 = np.dot(v2, v1)
        
        denom = d00 * d11 - d01 * d01
        
        # if the triangle is degenerate (zero area), fallback to a simple average
        if np.abs(denom) < 1e-12:
            avg_n = (na + nb + nc) / 3.0
            return avg_n / np.linalg.norm(avg_n)

        v = (d11 * d20 - d01 * d21) / denom
        w = (d00 * d21 - d01 * d20) / denom
        u = 1.0 - v - w
        
        # smoothly blend the normals
        interpolated_n = (u * na) + (v * nb) + (w * nc)
        
        # final normalization for the rotation matrix
        return interpolated_n / np.linalg.norm(interpolated_n)

    def makeIntersectionPoints(self) -> pv.PolyData:
        """
        Use slicing plane to get intersection points along trajectory path. 
        Plane is defined by 3 target points, with mid point defining direction.
        """
        vAB = self.targetPoints[2] - self.targetPoints[0]
        vAB = vAB / np.linalg.norm(vAB)

        vA2 = self.targetPoints[1] - self.targetPoints[0]
        vA2 = vA2 / np.linalg.norm(vA2)
    
        normal = np.cross(vAB, vA2) # get normal vector to plane defined by target points
        normal = normal / np.linalg.norm(normal)

        testPlane = pv.Plane(center=self.targetPoints[0], direction=normal, i_size=100, j_size=100)
        self.plotter.add_mesh(testPlane, color="cyan", show_edges=True, opacity=0.2, name='slicePlane',pickable=False)

        # get intersection of plane and mesh
        intersectionPoly = self.mesh.slice(normal=normal, origin=self.targetPoints[0])
        if intersectionPoly.n_points == 0:
            print("No intersection found between plane and mesh.")
            return None
        
        intersectionPoly = intersectionPoly.clean() # remove duplicate points

        return intersectionPoly

    def build_line_adjacency(self, poly):
        """Build adjacency list for line segments in the intersection polygon."""
        adjacency = {}
        lines = poly.lines
        i = 0
        while i < len(lines):
            n = int(lines[i])
            ids = lines[i + 1 : i + 1 + n]
            for a, b in zip(ids, ids[1:]):
                adjacency.setdefault(a, []).append(b)
                adjacency.setdefault(b, []).append(a)
            i += n + 1
        return adjacency

    def tracePath(self, points, adjacency, start_idx, next_idx, end_idx) -> list:
        """Trace a path from start_idx to end_idx through the adjacency graph, starting with next_idx."""
        path = [start_idx, next_idx]
        prev = start_idx
        cur = next_idx
        max_steps = len(points) * 2
        while cur != end_idx and len(path) < max_steps:
            neighbors = [n for n in adjacency.get(cur, []) if n != prev]
            if not neighbors:
                return None
            if len(neighbors) > 1:
                neighbors.sort(key=lambda n: np.linalg.norm(points[n] - points[end_idx]))
            prev = cur
            cur = neighbors[0]
            path.append(cur)
        return path if cur == end_idx else None

    def sortPoints(self, intersectionPoly, startPt, endPt, midPt) -> np.ndarray:
        """
        Sort intersection points to create a path from startPt to endPt, 
        using midPt to determine direction.
        """
        if intersectionPoly is None:
            return np.array([startPt])

        pointsRaw = np.array(intersectionPoly.points)
        if pointsRaw.shape[0] == 0:
            return np.array([startPt])

        adjacency = self.build_line_adjacency(intersectionPoly)

        tree = KDTree(pointsRaw)
        _, start_idx = tree.query(startPt)
        _, end_idx = tree.query(endPt)

        start_idx = int(start_idx)
        end_idx = int(end_idx)
        if start_idx == end_idx:
            return np.array([pointsRaw[start_idx]])

        vAB = midPt - startPt
        vAB = vAB / np.linalg.norm(vAB)

        neighbors = adjacency.get(start_idx, [])
        if not neighbors:
            return np.array([pointsRaw[start_idx]])

        candidate_paths = []
        for next_idx in neighbors:
            path = self.tracePath(pointsRaw, adjacency, start_idx, next_idx, end_idx)
            if path is not None:
                candidate_paths.append(path)

        if not candidate_paths:
            return np.array([pointsRaw[start_idx]])

        best_path = max(
            candidate_paths,
            key=lambda path: np.dot(
                (pointsRaw[path[1]] - pointsRaw[path[0]]) / np.linalg.norm(pointsRaw[path[1]] - pointsRaw[path[0]]),
                vAB,
            ),
        )

        ordered_points = pointsRaw[best_path]
        if not np.allclose(ordered_points[0], startPt):
            ordered_points[0] = startPt
        if not np.allclose(ordered_points[-1], endPt):
            ordered_points[-1] = pointsRaw[end_idx]

        return ordered_points

    def makeSplinePath(self, sortedPoints) -> np.ndarray:
        """Generate and visualize the spline path through the sorted intersection points."""
        self.tck, u = interpolate.splprep([sortedPoints[:, 0], sortedPoints[:, 1], sortedPoints[:, 2]], s=0, k=3)

        # evaluate spline at regular intervals to get smooth path
        self.uFine = np.linspace(0, 1, num=100) # adjust num for more/less points
        xFine, yFine, zFine = interpolate.splev(self.uFine, self.tck)
        pathFine = np.vstack((xFine, yFine, zFine)).T
        
        self.plotter.add_points(pathFine, color='blue', point_size=10, name='splinePath', pickable=False)
        
        return pathFine

    def animateDrill(self, speed_mms=2.0) -> None:
        """
        tck: The spline representation from splprep
        u_fine: The original u values used to generate the path
        speed_mms: Speed of the drill in mm/s
        """
        # calculate the physical distance along the spline
        points = np.array(interpolate.splev(self.uFine, self.tck)).T
        diffs = np.diff(points, axis=0)
        segment_lengths = np.sqrt((diffs**2).sum(axis=1))
        cumulative_dist = np.concatenate(([0], np.cumsum(segment_lengths)))
        total_length = cumulative_dist[-1]
        
        duration = total_length / speed_mms
        start_time = time.perf_counter()
        
        print(f"Path Length: {total_length:.2f}mm")

        while True:
            elapsed = time.perf_counter() - start_time
            if elapsed >= duration: 
                break

            # how far we should have traveled by now
            current_dist = elapsed * speed_mms
            
            # map distance back to the spline parameter 'u'
            # np.interp finds the 'u' value that corresponds to our current distance
            current_u = np.interp(current_dist, cumulative_dist, self.uFine)
            
            # get the precise 3D coordinate for that 'u'
            new_pos = interpolate.splev(current_u, self.tck)
            
            # at new point, get surface normal to orient drill
            surfaceNormal = self.getBarycentricNormal(new_pos)
            normal = surfaceNormal / np.linalg.norm(surfaceNormal)

            # get 45 degree rotation shift
            vel = np.array(interpolate.splev(current_u, self.tck, der=1))
            vel /= np.linalg.norm(vel)
            kVec = np.cross(normal, vel)
            kVec /= np.linalg.norm(kVec)
            kMat = np.array(
                ((0, -kVec[2], kVec[1]),
                (kVec[2], 0, -kVec[0]),
                (-kVec[1], kVec[0], 0))
            )
            angle = np.deg2rad(45)
            rot45 = (
                np.eye(3) + (np.sin(angle)*kMat) 
                + ( (1-np.cos(angle))*(kMat @ kMat))
            )

            # turn normal into rotation matrix for drill orientation
            up = [0, 0, 1]
            right = np.cross(up, normal)
            trueUp = np.cross(normal, right)
            origRot = np.eye(3)
            origRot[0:3, 0] = right
            origRot[0:3, 1] = trueUp
            origRot[0:3, 2] = normal

            rTot = origRot @ rot45 # shift by 45 degrees for ideal drilling angle

            transform = np.eye(4)
            transform[0:3, 0:3] = rTot
            transform[0:3, 3] = np.array([new_pos[0], new_pos[1], new_pos[2]])

            # update and render
            self.drillActor.user_matrix = transform
            self.plotter.render()

    def clickCallback(self, point):
        if point is not None:
            print("Selected point: ", point)
            self.targetPoints[self.numPts] = point
            self.numPts += 1
            self.plotter.add_points(self.targetPoints, color='green', point_size=5, name='targetPoints', pickable=False)

            if self.numPts == 3:
                print("Target points selected, generating path...")
                self.run()
                self.draw()

    def resetVisual(self) -> None:
        self.plotter.remove_actor('slicePlane')
        self.plotter.remove_actor('splinePath')
        self.plotter.remove_actor('targetPoints')
        self.targetPoints = np.zeros((3, 3))
        self.numPts = 0

    def draw(self) -> None:
        # update visualization
        self.plotter.show_axes()
        self.plotter.show()

    def run(self) -> None:
        """Main function to generate drilling path and animate drill based on selected target points."""
        intersectionPoly = self.makeIntersectionPoints()
        sortedPoints = self.sortPoints(intersectionPoly, self.targetPoints[0], self.targetPoints[2], self.targetPoints[1])
        splinePath = self.makeSplinePath(sortedPoints)
        self.animateDrill()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("mesh_input", help="Path to the input mesh file (required)")
    parser.add_argument("--meters2mm", action="store_true", help="Convert mesh units from meters to millimeters")
    args = parser.parse_args()

    if args.mesh_input is None:
        print("Error: Mesh input is required.")
        parser.print_help()
        exit(1)
    
    scale = 1.0
    if args.meters2mm:
        scale = 1000.0

    testGen = trajectoryGenerator(args.mesh_input, scale=scale)
    testGen.draw()