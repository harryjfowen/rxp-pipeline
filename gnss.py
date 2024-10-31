import os
import json
import numpy as np
import pandas as pd
import glob 
import argparse

###SCRIPT TO GENERATE A MATRIX FROM GNSS HSON FILES TO FEED THE RXP2PLY GLOBAL_MATRIX ARGUMENT

def collect_gnss(folder_path, fields):

    pose_files = glob.glob(f'{folder_path}/**/*{".pose"}', recursive=True)
    if len(pose_files) == 0:
        raise Exception(f'No pose files found in {folder_path}')
    data = []
    for file_name in pose_files:
        with open(file_name) as f:
            json_data = json.load(f)
            row = []
            row.append(os.path.splitext(os.path.basename(file_name))[0])
            for field in fields:
                row.append(json_data["gnss"][field])
            data.append(row)
    gnss = pd.DataFrame(data, columns=['SCANPOS','Latitude', 'Longitude', 'Positional Accuracy', 'Elevation', 'EPSG'])
    return gnss, pose_files

def collect_origins(pose_files, matrix_folder):

    if not os.path.isdir(matrix_folder): raise Exception(f'no such directory: {matrix_folder}')
    n_scans = len(pose_files)
    origins = np.zeros((n_scans, 4))
    origins[:, -1] = 1
    origins = pd.DataFrame(origins, columns=['x', 'y', 'z', 'affine'])
    matrices_files = [f for f in os.listdir(matrix_folder) if f.endswith('.DAT')]
    for i in range(len(matrices_files)):
        if matrices_files[i].endswith('.DAT'):
            transform_matrix = np.loadtxt(matrices_files[i])
            current_origin = origins.iloc[i, origins.columns.isin(['x', 'y', 'z', 'affine'])]
            origins.iloc[i, origins.columns.isin(['x', 'y', 'z', 'affine'])] = np.matmul(transform_matrix, current_origin.T).T
    return origins

def collect_transformation_matrix(gnss, origins):
    
    gnss_pos = gnss[['Latitude', 'Longitude', 'Elevation']].to_numpy()
    origin_pos = origins[['x', 'y', 'z']].to_numpy()
    aligned_pos = gnss_pos
    ones_col = np.ones((len(origin_pos), 1))
    A = np.hstack((origin_pos, ones_col))
    B = aligned_pos
    X, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
    T = np.eye(4)
    T[:3, :] = X.T
    return T

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--project', '-p', required=True, type=str, help='path to .PROJ folder')
    parser.add_argument('--odir', type=str, default='.', help='output directory')
    parser.add_argument('--verbose', action='store_true', help='print something')

    args = parser.parse_args()

    if not args.odir: args.odir = args.project

    args.matrix_dir = os.path.join(args.project, 'matrix')
    args.fields = ['latitude', 'longitude', 'positionAccuracy', 'altitude', 'coordinateSystem']

    gnss, pose_files = collect_gnss(args.project, args.fields)
    origins = collect_origins(pose_files, args.matrix_dir)
    global_matrix = collect_transformation_matrix(gnss, origins)

    np.savetxt(args.odir+'/global_matrix.DAT', global_matrix, delimiter=' ', fmt='%1.16f')

