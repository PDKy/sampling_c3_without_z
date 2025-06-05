import math
import trimesh as tm
import numpy as np
import os
from trimesh.triangles import mass_properties

# x_vertical = 0.16
# y_vertical = 0.04
# z_vertical = 0.04
# x_horizontal = 0.04
# y_horizontal = 0.16
# z_horizontal = z_vertical
# x_tot = x_vertical + x_horizontal
# y_tot = y_horizontal
# z_tot = z_vertical
target_name = "bear"


filename = f'examples/sampling_c3/urdf/mesh/{target_name}/textured.obj'
mesh = tm.load_mesh(filename)

if hasattr(mesh, 'vertex_normals'):
    mesh.vertex_normals = None
if hasattr(mesh, 'face_normals'):
    mesh.face_normals = None

bounding_mesh_array = mesh.bounds


mesh.density = 1000

mass_prop = mesh.mass_properties

I = mass_prop["inertia"]
mass = mass_prop['mass']
ixx, ixy, ixz = I[0]
_,   iyy, iyz = I[1]
_,    _,  izz = I[2]

xc = mass_prop["center_mass"][0]
yc = mass_prop["center_mass"][1]
zc = mass_prop["center_mass"][2]

print(mass_prop["center_mass"])


print(bounding_mesh_array)

x_bound = np.abs(bounding_mesh_array[0, 0]) + np.abs(bounding_mesh_array[1, 0])
y_bound = np.abs(bounding_mesh_array[0, 1]) + np.abs(bounding_mesh_array[1, 1])
# z_bound = bounding_mesh_array[0, 2]
z_bound = 0

print(x_bound, y_bound, z_bound)



sdf = f"""<?xml version="1.0"?>
<sdf version="1.7">
    <model name="push_anything"> 
    
        <link name="body">
            <inertial>
                <pose>{xc} {yc} {zc} 0 0 0</pose>
                
                <mass> {mass} </mass>
                
                <inertia>
                    <ixx>{ixx}</ixx>
                    <ixy>{ixy}</ixy>
                    <ixz>{ixz}</ixz>
                    <iyy>{iyy}</iyy>
                    <iyz>{iyz}</iyz>
                    <izz>{izz}</izz>
                </inertia>
            </inertial>
            
            <visual name="body">
                <geometry>
                <mesh>
                    <uri>mesh/{target_name}/textured.obj</uri>
                </mesh>
                </geometry>
                </visual>
                
            <collision name = "body_volume">
                <geometry>
                    <mesh>
                        <uri>mesh/{target_name}/textured.obj</uri>
                        <drake:declare_convex/>
                    </mesh>
                </geometry>
                <drake:proximity_properties>
                <drake:compliant_hydroelastic/>
                <drake:hydroelastic_modulus> 3.0e7 </drake:hydroelastic_modulus>
                <drake:mesh_resolution_hint> 0.18 </drake:mesh_resolution_hint>
                <drake:hunt_crossley_dissipation>10</drake:hunt_crossley_dissipation>
                <drake:mu_dynamic>0.3</drake:mu_dynamic>
                </drake:proximity_properties>
                
            </collision>
            
            
"""

corner_positions = [
    (x_bound, 0, z_bound),
    (-x_bound, -y_bound, z_bound),
    (-x_bound, y_bound, z_bound)
]

parent_links = [
    "horizontal_link",
    "horizontal_link",
    "vertical_link"
]

for i, (pos, parent) in enumerate(zip(corner_positions,parent_links)):
    corner_name = f"corner_{i}"
    sdf += f"""

    <collision name="{corner_name}">
        <geometry>
          <sphere>
            <radius>0.001</radius>
          </sphere>
        </geometry>
        <drake:proximity_properties>
        <!-- <drake:compliant_hydroelastic/>
        <drake:hydroelastic_modulus> 3.0e7 </drake:hydroelastic_modulus>
        <drake:mesh_resolution_hint> 0.18 </drake:mesh_resolution_hint>
        <drake:hunt_crossley_dissipation>10</drake:hunt_crossley_dissipation> -->
        <drake:mu_dynamic>0.3</drake:mu_dynamic>
        </drake:proximity_properties>
        <pose> {pos[0]} {pos[1]} {pos[2]} 0 0 0</pose>
      </collision>
      
    <visual name="{corner_name}">
        <geometry>
          <sphere>
            <radius>0.001</radius>
          </sphere>
        </geometry>
        <material>
        <diffuse>1 0 0 1</diffuse>
        </material>
        <pose> {pos[0]} {pos[1]} {pos[2]} 0 0 0</pose>
      </visual>


"""
sdf += "\n</link>\n"
sdf += "\n</model>\n"
sdf += "\n</sdf>\n"


out_dir = "/home/dair/opt/sampling_based_c3/dairlib/examples/sampling_c3/urdf"

os.makedirs(out_dir, exist_ok=True)

out_path = os.path.join(out_dir, "target.sdf")


with open(out_path, "w") as f:
    f.write(sdf)

#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#generate sim sdf


sdf_sim = f"""<?xml version="1.0"?>
<sdf version="1.7">
    <model name="push_anything"> 
    
        <link name="body">
            <inertial>
                <pose>0 0 0 0 0 0</pose>
                
                <mass> {mass} </mass>
                
                <inertia>
                    <ixx>{ixx}</ixx>
                    <ixy>{ixy}</ixy>
                    <ixz>{ixz}</ixz>
                    <iyy>{iyy}</iyy>
                    <iyz>{iyz}</iyz>
                    <izz>{izz}</izz>
                </inertia>
            </inertial>
            
            <visual name="body">
                <geometry>
                <mesh>
                    <uri>mesh/{target_name}/textured.obj</uri>
                </mesh>
                </geometry>
                </visual>
                
            <collision name = "body_volume">
                <geometry>
                    <mesh>
                        <uri>mesh/{target_name}/textured.obj</uri>
                        <drake:declare_convex/>
                    </mesh>
                </geometry>
                <drake:proximity_properties>
                <drake:compliant_hydroelastic/>
                <drake:hydroelastic_modulus> 3.0e7 </drake:hydroelastic_modulus>
                <drake:mesh_resolution_hint> 0.18 </drake:mesh_resolution_hint>
                <drake:hunt_crossley_dissipation>10</drake:hunt_crossley_dissipation>
                <drake:mu_dynamic>0.3</drake:mu_dynamic>
                </drake:proximity_properties>
                
            </collision>   
        </link>
"""


sdf_sim += "\n</model>\n"
sdf_sim += "\n</sdf>\n"


out_dir = "/home/dair/opt/sampling_based_c3/dairlib/examples/sampling_c3/urdf"

os.makedirs(out_dir, exist_ok=True)

out_path = os.path.join(out_dir, "target_sim.sdf")


with open(out_path, "w") as f:
    f.write(sdf_sim)