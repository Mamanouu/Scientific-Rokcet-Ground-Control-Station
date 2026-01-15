"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
"""
import sys
import numpy as np
import pyqtgraph.opengl as gl
from rotations import Euler2Rotation, Euler2Quaternion, Quaternion2Rotation


class DrawMav:
    def __init__(self, state, window):
        """
        Draw the MAV.

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.north  # north position
            state.east  # east position
            state.altitude   # altitude
            state.phi  # roll angle
            state.theta  # pitch angle
            state.psi  # yaw angle
        """
        # get points that define the non-rotated, non-translated mav and the mesh colors
        self.mav_points, self.mav_meshColors = self.get_points()

        mav_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of mav as a rotation matrix R from body to inertial
        R = Quaternion2Rotation(np.array([[1.0], [0.0], [0.0], [0.0]])) # neutral quaternion
        # rotate and translate points defining mav
        rotated_points = self.rotate_points(self.mav_points, R)
        translated_points = self.translate_points(rotated_points, mav_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points)
        self.mav_body = gl.GLMeshItem(vertexes=mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=self.mav_meshColors,  # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=True,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering
        #self.mav_body.setGLOptions('translucent')
        # ============= options include
        # opaque        Enables depth testing and disables blending
        # translucent   Enables depth testing and blending
        #               Elements must be drawn sorted back-to-front for
        #               translucency to work correctly.
        # additive      Disables depth testing, enables blending.
        #               Colors are added together, so sorting is not required.
        # ============= ======================================================
        window.addItem(self.mav_body)  # add body to plot
        # default_window_size = (500, 500)
        # window.resize(*default_window_size)


    def update(self, state):
        mav_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates

        Q = np.array([[state.q0], [state.q1], [state.q2], [state.q3]])
        # attitude of mav as a rotation matrix R from body to inertial
        R = Quaternion2Rotation(Q)
        # rotate and translate points defining mav
        rotated_points = self.rotate_points(self.mav_points, R)
        translated_points = self.translate_points(rotated_points, mav_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points)
        # draw MAV by resetting mesh using rotated and translated points
        self.mav_body.setMeshData(vertexes=mesh, vertexColors=self.mav_meshColors)

    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
        return translated_points

    def get_points(self):
        """"
            Points that define the mav, and the colors of the triangular mesh
            Define the points on the aircraft following diagram in Figure C.3
        """
        ##### TODO ####
        # Define the points on the aircraft following diagram Fig 2.14
        # points are in NED coordinates
        ##### TODO #####
        points = np.array([[0,  0,  -8],  # Apex of the pyramid
                           [2.5, -2.5, 0],  # Base corner 1
                           [-2.5, -2.5, 0],   # Base corner 2
                           [-2.5, 2.5, 0],    # Base corner 3
                           [2.5, 2.5, 0], # Base corner 4
                           [2.5, 2.5, 20],
                           [-2.5, 2.5, 20],
                           [-2.5, -2.5, 20],
                           [2.5, -2.5, 20],
                           [2.5, 0, 15], # fin 1
                           [6, 0, 20], # fin 1
                           [2.5, 0, 20], # fin 1
                           [-2.5, 0, 15], # fin 2 symetric to fin 1 % plane YZ
                           [-6, 0, 20], # fin 2
                           [-2.5, 0, 20], # fin 2
                           [0, 2.5, 15], # fin 3 
                           [0, 6, 20], # fin 3
                           [0, 2.5, 20], # fin 3
                           [0, -2.5, 15], # fin 4 symetric to fin 3 % plane XZ
                           [0, -6, 20], # fin 4
                           [0, -2.5, 20], # fin 4
                           ]   
                          ).T

        scale = 0.75
        points = points * scale

        #   define the colors for each face of triangular mesh
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        white = np.array([255., 255., 255., 1])
        meshColors = np.empty((18, 3, 4), dtype=np.float32)

        # Assign colors for each mesh section
        ##### TODO #####
        meshColors[0] = blue # nose-top
        meshColors[1] = blue # nose-right
        meshColors[2] = blue # nose-bot
        meshColors[3] = blue # nose-left
        meshColors[4] = white # fuse-top
        meshColors[5] = white # fuse-right
        meshColors[6] = white # fuse-bot
        meshColors[7] = white # fuse-left
        meshColors[8] = white # wing-le
        meshColors[9] = white # wing-te
        meshColors[10] = white # tail-le
        meshColors[11] = white # tail-te
        meshColors[12] = white # bottom
        meshColors[13] = white # bottom
        meshColors[14] = blue # face fin
        meshColors[15] = blue # back fin
        meshColors[16] = blue # face fin
        meshColors[17] = blue # back fin
        

        return points, meshColors

    def points_to_mesh(self, points):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T

        #Define each section of the mesh with 3 points
        ##### TODO #####
        mesh = np.array([[points[0], points[1], points[2]],  # cone 1
                         [points[0], points[2], points[3]],  # cone 2
                         [points[0], points[3], points[4]],  # cone 3
                         [points[0], points[4], points[1]],  # cone 4
                         [points[4], points[5], points[6]],  # fuse right upper triangle
                         [points[6], points[3], points[4]],  # fuse right upper triangle
                         [points[3], points[6], points[2]],  # fuse back upper triangle
                         [points[2], points[7], points[6]],  # fuse back lower triangle
                         [points[7], points[2], points[1]],  # fuse left upper triangle
                         [points[1], points[7], points[8]],  # fuse left lower triangle
                         [points[8], points[1], points[5]],  # fuse face lower triangle
                         [points[5], points[4], points[1]],  # fuse face upper triangle
                         [points[7], points[6], points[5]],  # bottom back triangle
                         [points[7], points[8], points[5]],  # bottom face triangle
                         [points[9], points[10], points[11]], # face fin
                         [points[12], points[13], points[14]], # back fin
                         [points[15], points[16], points[17]], # right fin
                         [points[18], points[19], points[20]], # left fin
                         ])
        return mesh
