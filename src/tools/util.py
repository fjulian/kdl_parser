from scipy.spatial.transform import Rotation as R
import numpy as np

def quat_from_rpy(orient_rpy):
    orient_rep = R.from_euler("xyz", orient_rpy.tolist())
    return orient_rep.as_quat()


def quat_from_rpy_yaw_first(orient_rpy):
    orient_rep = R.from_euler("zxy", orient_rpy.tolist())
    return orient_rep.as_quat()


def quat_from_rotvec(rotvec):
    orient_rep = R.from_rotvec(rotvec)
    return orient_rep.as_quat()


def quat_from_mat(mat):
    orient_rep = R.from_dcm(mat)
    return orient_rep.as_quat()

def rotate_orient(orig, axis="z", deg=0.0):
    r = R.from_quat(orig)
    assert axis=='x' or axis=='y' or axis=='z', "Invalid axis"
    op = R.from_euler(axis, deg, degrees=True)
    res = op * r
    return res.as_quat()

def homogenous_trafo(translation, rotation):
    T = np.concatenate((rotation.as_dcm(), translation.reshape(-1,1)), axis=1)
    T = np.concatenate((T, np.array([[0.0, 0.0, 0.0, 1.0]])), axis=0)
    return T

def invert_hom_trafo(hom_trafo):
    res = np.eye(4)
    res[:3,:3] = np.transpose(hom_trafo[:3,:3])
    res[:3,3] = - np.matmul(np.transpose(hom_trafo[:3,:3]), hom_trafo[:3,3])
    return res

class IKError(Exception):
    pass

class ObjectInfo:
    def __init__(self, urdf_path_, init_pos_, init_orient_, init_scale_=1.0, grasp_pos_=[], grasp_orient_=[], model_=None, nav_angle_=None):
        self.urdf_path = urdf_path_
        self.init_pos = init_pos_
        self.init_orient = init_orient_
        self.scale = init_scale_
        self.grasp_pos = grasp_pos_
        self.grasp_orient = grasp_orient_
        self.model = model_
        self.nav_angle = nav_angle_
