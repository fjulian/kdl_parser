from scipy.spatial.transform import Rotation as R


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

class IKError(Exception):
    pass
