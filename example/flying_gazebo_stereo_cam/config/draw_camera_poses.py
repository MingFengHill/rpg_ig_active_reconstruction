import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import numpy as np
import open3d as o3d
import matplotlib.tri as mtri
from scipy.spatial.transform import Rotation as R


def get_camera_wireframe(scale: float = 0.3):  # pragma: no cover
    """
    Returns a wireframe of a 3D line-plot of a camera symbol.
    """
    scale = 0.1
    a = scale * np.asarray([-2, 1.5, -4])
    up1 = scale * np.asarray([0, 1.5, -4])
    up2 = scale * np.asarray([0, 2, -4])
    b = scale * np.asarray([2, 1.5, -4])
    c = scale * np.asarray([-2, -1.5, -4])
    d = scale * np.asarray([2, -1.5, -4])
    C = np.zeros(3)
    F = scale * np.asarray([0, 0, -3])
    camera_points = np.asarray([a, up1, up2, up1, b, d, c, a, C, b, d, C, c, C, F], dtype='float')
    camera_points = camera_points/2
    camera_points = camera_points.T
    camera_points = np.row_stack((camera_points, np.asarray([1 for i in range(np.shape(camera_points)[1])])))
    return camera_points


def draw_camera(camera_pose, ax):
    cam_wires_canonical = get_camera_wireframe()
    rlt = np.matmul(camera_pose, cam_wires_canonical)
    rlt = rlt[0:3]
    rlt = rlt.T
    for k in range(np.shape(rlt)[0] - 1):
        ax.plot([rlt[k][0], rlt[k + 1][0]],
                [rlt[k][1], rlt[k + 1][1]],
                [rlt[k][2], rlt[k + 1][2]],
                color="#FF7D1E", linewidth=0.8)


def split_ig_poses_config(line_input):
    output = []
    cur_str = ""
    for cur in line_input:
        if cur == " ":
            if cur_str != "":
                output.append(float(cur_str))
                cur_str = ""
        else:
            cur_str += cur
    if cur_str != "":
        output.append(float(cur_str))
    return output


def get_camera_pose(camera_pose):
    position = [camera_pose[0], camera_pose[1], camera_pose[2]]
    quaternion = [camera_pose[3], camera_pose[4], camera_pose[5], camera_pose[6]]
    r = R.from_quat(quaternion)
    rotation_matrix = r.as_matrix()
    transformation = [[rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], position[0]],
                        [rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], position[1]],
                        [rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], position[2]],
                        [0, 0, 0, 1]]
    return transformation


if __name__ == '__main__':
    Axes3D = Axes3D  # pycharm auto import
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # draw mesh using open3d & matplot
    mesh = o3d.io.read_triangle_mesh('./model.obj')
    mesh.compute_vertex_normals()
    # o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True)
    vertices = np.asarray(mesh.vertices)
    vertices_t = vertices.T
    triang = mtri.Triangulation(vertices_t[0], vertices_t[1], np.asarray(mesh.triangles))
    # ax.plot_trisurf(triang, vertices_t[2], alpha=0.3, linewidth=0, antialiased=False)

    with open("./poses.txt", 'r') as f:
        lines = f.readlines()
        for line in lines:
            line = line.strip()
            line = split_ig_poses_config(line)
            if len(line) < 2:
                continue
            # ax.scatter(line[0], line[1], line[2], color="#FF7D1E")
            pose = get_camera_pose(line)
            draw_camera(pose, ax)

    ax.set_xlabel("x")
    ax.set_ylabel("z")
    ax.set_zlabel("y")
    plt.show()
