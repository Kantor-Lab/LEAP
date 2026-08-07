"""
Random file for testing the transformation from GPS in UTM to local coordinates.
"""


import numpy as np
from pyproj import Transformer
from plyfile import PlyData, PlyElement


def test(lat, lot):
    # Test the math
    T = np.array([[-0.383874421, -0.923385309, 0.000000000, 4360743.186303443],
                  [0.923385309, -0.383874421, 0.000000000, 1174557.450282508],
                  [0.000000000, 0.000000000, 1.000000000, -234.041032714],
                  [0.000000000, 0.000000, 0.000000000, 1.000000000]])
    R = T[:3, :3]
    t = T[:3, 3]
    zone_number = 17
    hemisphere = 'N'
    epsg = 32600 + zone_number if hemisphere == 'N' else 32700 + zone_number
    to_utm = Transformer.from_crs('EPSG:4326', f'EPSG:{epsg}', always_xy=True)
    easting, northing = to_utm.transform(lot, lat)
    altitude = 0.0
    p_utm = np.array([easting, northing, altitude, 1])
    p_local = T @ p_utm
    print(f'lat={lat}, lon={lot} -> easting={easting}, northing={northing}, -> p_local={p_local}')

import numpy as np
from plyfile import PlyData, PlyElement

def transform_point_cloud(input_ply_path, output_ply_path, to_local=True):
    print(f"Loading {input_ply_path}...")
    plydata = PlyData.read(input_ply_path)
    vertex_data = plydata['vertex'].data
    
    points = np.vstack((vertex_data['x'], vertex_data['y'], vertex_data['z'])).astype(np.float64).T
    ones = np.ones((points.shape[0], 1))
    points_h = np.hstack((points, ones))
    
    T = np.array([[-0.383874421, -0.923385309, 0.000000000, 4360743.186303443],
                  [0.923385309, -0.383874421, 0.000000000, 1174557.450282508],
                  [0.000000000, 0.000000000, 1.000000000, -234.041032714],
                  [0.000000000, 0.000000, 0.000000000, 1.000000000]])
    T_inv = np.linalg.inv(T)
    
    matrix_to_apply = T if to_local else T_inv
    direction = "UTM -> Local" if to_local else "Local -> UTM"
    print(f"Applying {direction} transform...")
    
    transformed_points_h = (matrix_to_apply @ points_h.T).T
    
    vertex_data['x'] = transformed_points_h[:, 0]
    vertex_data['y'] = transformed_points_h[:, 1]
    vertex_data['z'] = transformed_points_h[:, 2]
    
    print(f"Saving to {output_ply_path}...")
    PlyData([PlyElement.describe(vertex_data, 'vertex')], text=False).write(output_ply_path)
    print("Done!")

test(40.4429915283333, -79.94507359333333)
transform_point_cloud("cmu_utm.ply", "cmu_local_test.ply", to_local=True)