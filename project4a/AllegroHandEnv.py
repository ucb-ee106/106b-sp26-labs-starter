import numpy as np
import dm_control
import mujoco as mj

class AllegroHandEnv:
    def __init__(self, physics: dm_control.mjcf.physics.Physics, 
                 q_h_slice: slice, 
                 object_name: str, 
                 num_fingers=4):
        self.physics = physics
        self.q_h_slice = q_h_slice
        self.num_fingers = num_fingers
        self.object_name = object_name

    def set_configuration(self, q_h: np.array):
        self.physics.data.qpos[self.q_h_slice] = q_h
        self.physics.forward()

    def get_fingertip_positions(self, fingertip_names: list[str]):
        """
        Input: list of fingertip names in the XML
        Returns: (num_fingers x 3) np.array containing 
        finger positions in workspace coordinates
        """
        #YOUR CODE HERE

    def get_fingertip_normals(self, contact: mj._structs._MjContactList):
        """
        Input: contact data structure that contains MuJoCo contact information
        Returns the normal vector for each finger that's in contact with the ball
        
        Tip: See information about the mjContact_ struct here: https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjcontact
        """
        #YOUR CODE HERE

class AllegroHandEnvSphere(AllegroHandEnv):
    def __init__(self, physics: dm_control.mjcf.physics.Physics, 
                 sphere_center: int, 
                 sphere_radius: int, 
                 q_h_slice: slice, 
                 object_name: str):
        super().__init__(physics, q_h_slice, object_name)
        self.physics = physics
        self.sphere_center = sphere_center
        self.sphere_radius = sphere_radius
        self.q_h_slice = q_h_slice
        self.num_fingers = 4
    
    def sphere_surface_distance(self, pos: np.array, center: np.array, radius: int):
        """
        Returns the distance from pos to the surface of a sphere with a specified
        radius and center
        """
        d = np.linalg.norm(pos - center) - radius
        return d