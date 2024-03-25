import pymeshlab

def calculate_inertial_tag(filename=None, mass=-1, pr=8, scale_factor=100):
    ms = pymeshlab.MeshSet()

    if filename is None:
        print("file name?")
        filename = input()
    ms.load_new_mesh(filename)

    if mass < 0:
        print("mass?")
        mass = float(input())
    
    print("calculating center of mass")
    geom = ms.get_geometric_measures()
    com = geom["barycenter"]

    print("scaling the mesh")
    ms.compute_matrix_from_scaling_or_normalization(axisx=scale_factor, axisy=scale_factor, axisz=scale_factor)

    print("generating the convex hull of the mesh")
    ms.generate_convex_hull()

    print("calculating inertia tensor")
    geom=ms.get_geometric_measures()
    volume = geom['mesh_volume']
    tensor = geom['inertia_tensor'] / pow(scale_factor, 2)*mass/volume
    inertia_xml = f'<inertial>\n  <origin xyz= "{com[0]:.{pr}f} {com[1]:.{pr}f} {com[2]:.{pr}f}"/>\n  <mass value = "{mass:.{pr}f}"/>\n  <ixx = "{tensor[0,0]:.{pr}f}" ixy = "{tensor[1,0]:.{pr}f}" ixz = "{tensor[2,0]:.{pr}f}" iyy = "{tensor[1,1]:.{pr}f}" iyz = "{tensor[1,2]:.{pr}f}" izz = "{tensor[2,2]:.{pr}f}"/>\n </inertial>'
    print(inertia_xml)

if __name__=='__main__':
    calculate_inertial_tag() 
