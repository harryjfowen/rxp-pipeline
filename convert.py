import sys, os, glob, multiprocessing, json, argparse, math, csv
import pandas as pd
import numpy as np
import plyio
import pdal
from pathlib import Path
from collections import defaultdict
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
import re

class PointCloudDownsampler:
    """Handles point cloud downsampling operations"""
    
    def __init__(self, pc: pd.DataFrame, vlength: float):
        """
        Initialize downsampler with point cloud and voxel size
        
        Args:
            pc: Point cloud data as DataFrame
            vlength: Voxel size for downsampling
        """
        self.indices = pc.index
        self.pc = pc.values
        self.vlength = vlength

    def voxelisation_by_reflectance(self) -> np.ndarray:
        """
        Downsample keeping point with highest reflectance in each voxel
        
        Returns:
            Array of indices for selected points
        """
        voxel_indices = np.floor(self.pc[:, :3] / self.vlength).astype(int)
        voxel_dict = defaultdict(list)
        
        for i, voxel_index in enumerate(voxel_indices):
            voxel_dict[tuple(voxel_index)].append(i)
            
        selected_indices = [
            voxel_points_indices[np.argmax(self.pc[voxel_points_indices, 3])]
            for voxel_points_indices in voxel_dict.values()
        ]
        
        return self.indices[selected_indices]

    def random_voxelisation(self) -> np.ndarray:
        """
        Downsample keeping random point in each voxel
        
        Returns:
            Array of indices for selected points
        """
        voxel_indices = np.floor(self.pc[:, :3] / self.vlength).astype(int)
        voxel_dict = defaultdict(list)
        
        for i, voxel_index in enumerate(voxel_indices):
            voxel_dict[tuple(voxel_index)].append(i)
            
        selected_indices = [
            voxel_points_indices[np.random.randint(len(voxel_points_indices))]
            for voxel_points_indices in voxel_dict.values()
        ]
        
        return self.indices[selected_indices]
    
class PointCloudProcessor:
    """Handles point cloud processing operations"""
    def __init__(self, args):
        self.args = args
        self._setup_directories()
        self.setup_processing_env()
        
    def _setup_directories(self):
        """Setup input and output directories"""
        self.args.project = Path(self.args.project)
        
        if self.args.odir == '.':
            self.args.odir = self.args.project / 'clouds'
            if self.args.verbose:
                print(f"Using default output directory: {self.args.odir}")
        else:
            self.args.odir = Path(self.args.odir)
            
        self.args.odir.mkdir(parents=True, exist_ok=True)
        
        self.args.matrix_dir = self.args.project / 'matrix'
        if not self.args.matrix_dir.is_dir():
            raise Exception(f'No such directory: {self.args.matrix_dir}')
        
    def setup_processing_env(self):
        """Setup processing environment and parameters"""
        self.args.matrix_dir = os.path.join(self.args.project, 'matrix')
        if not os.path.isdir(self.args.matrix_dir):
            raise Exception(f'no such directory: {self.args.matrix_dir}')
        os.makedirs(self.args.odir, exist_ok=True)
        
        self.args.ScanPos = sorted(glob.glob(os.path.join(self.args.project, f'{self.args.prefix}*')))
        if self.args.plot and not self.args.plot_code:
            self.args.plot_code = os.path.basename(self.args.project.rstrip('/'))[:5]
        
        self.setup_spatial_params()
        
    def setup_spatial_params(self):
        """Setup spatial parameters including bbox and hull"""
        M = glob.glob(os.path.join(self.args.matrix_dir, f'{self.args.prefix}*.*'))
        if not M:
            raise Exception('no matrix files found')
            
        matrix_arr = np.zeros((len(M), 3))
        with open(os.path.join(self.args.odir, 'scan_positions.txt'), 'w') as f:
            for i, m in enumerate(M):
                matrix_arr[i, :] = np.loadtxt(m)[:3, 3]
                f.write(f'{matrix_arr[i, 0]},{matrix_arr[i, 1]},{1.3}\n')
        
        hull = ConvexHull(matrix_arr[:, :2])
        polygon = Polygon(matrix_arr[hull.vertices, :2])
        self.args.convex_hull_wkt = (polygon.buffer(self.args.buffer) if not self.args.invert 
            else polygon.buffer(self.args.buffer).difference(polygon.buffer(-self.args.buffer))).wkt
        
        self.args.buffer = 10
        xmin = math.floor((matrix_arr[:, 0].min() - self.args.buffer) / self.args.tile) * self.args.tile
        xmax = math.ceil((matrix_arr[:, 0].max() + self.args.buffer) / self.args.tile) * self.args.tile
        ymin = math.floor((matrix_arr[:, 1].min() - self.args.buffer) / self.args.tile) * self.args.tile
        ymax = math.ceil((matrix_arr[:, 1].max() + self.args.buffer) / self.args.tile) * self.args.tile
        
        max_dim = max(xmax - xmin, ymax - ymin)
        self.args.bbox = [xmin, xmin + max_dim, ymin, ymin + max_dim]
        
        if self.args.global_matrix:
            self.args.global_matrix = np.loadtxt(self.args.global_matrix)
            bbox_points = np.array([[self.args.bbox[0], self.args.bbox[2], 0, 1],
                                  [self.args.bbox[1], self.args.bbox[2], 0, 1],
                                  [self.args.bbox[0], -1000, 0, 1],
                                  [self.args.bbox[1], -1000, 0, 1]])
            transformed = np.dot(self.args.global_matrix, bbox_points.T).T
            self.args.bbox = [np.min(transformed[:, 0]), np.max(transformed[:, 0]),
                            np.min(transformed[:, 1]), np.max(transformed[:, 1])]
        else:
            self.args.global_matrix = np.identity(4)
        
        X, Y = np.meshgrid(
            np.arange(math.floor((self.args.bbox[0] - self.args.buffer) / self.args.tile) * self.args.tile,
                     self.args.bbox[1] + self.args.tile + self.args.buffer, self.args.tile),
            np.arange(math.floor((self.args.bbox[2] - self.args.buffer) / self.args.tile) * self.args.tile,
                     self.args.bbox[3] + self.args.tile + self.args.buffer, self.args.tile))
        
        self.args.tiles = pd.DataFrame(data=np.vstack([X.flatten(), Y.flatten()]).T.astype(int),
                                     columns=['x', 'y'])
        self.args.tiles['tile'] = range(len(self.args.tiles))
        self.args.tiles = self.args.tiles[['x', 'y', 'tile']].reset_index()
        self.args.n = len(str(len(self.args.tiles)))

    def _classify_ground(self, input_ply: str):
        """Classify ground points and calculate normalized heights for a complete point cloud"""
        if self.args.verbose:
            print("Performing ground classification on complete point cloud...")
        
        base_name = input_ply.replace('.ply', '')
        
        pipeline_stages = [
            {
                "type": "readers.ply",
                "filename": input_ply
            },
            {
                "type": "filters.csf",
                "resolution": self.args.cloth_resolution,
                "threshold": self.args.class_threshold,
                "rigidness": self.args.rigid,
                "smooth": self.args.slope_smooth
            },
            {
                "type": "filters.hag_nn",
                "count": 1
            }
        ]
        
        pipeline = pdal.Pipeline(json.dumps(pipeline_stages))
        pipeline.execute()
        
        arr = pipeline.arrays[0]
        df = pd.DataFrame(arr)
        
        ground_mask = df['Classification'] == 2
        
        ground_points = df[ground_mask][['X', 'Y', 'Z', 'Reflectance']].rename(
            columns={'X': 'x', 'Y': 'y', 'Z': 'z', 'Reflectance': 'reflectance'})
        
        if len(ground_points) > 0:
            ground_file = f"{base_name}_ground.ply"
            plyio.write_ply(ground_file, ground_points)
            if self.args.verbose:
                print(f"Saved ground points to: {ground_file}")
        
        non_ground_points = df[~ground_mask][['X', 'Y', 'Z', 'Reflectance', 'HeightAboveGround']].rename(
            columns={'X': 'x', 'Y': 'y', 'Z': 'z', 
                    'Reflectance': 'reflectance',
                    'HeightAboveGround': 'n_z'})
        
        if len(non_ground_points) > 0:
            non_ground_file = f"{base_name}.ply"
            plyio.write_ply(non_ground_file, non_ground_points)
            if self.args.verbose:
                print(f"Saved normalized non-ground points to: {non_ground_file}")
            
        return ground_file, non_ground_file

    def process_scan(self, scan_pos):
            """Process single scan position"""
            base, scan = os.path.split(scan_pos)
            
            try:
                if self.args.test:
                    rxp_pattern = os.path.join(base, scan, 'scans' if base.endswith('.PROJ') else '', '??????_??????.mon.rxp')
                else:
                    rxp_pattern = os.path.join(base, scan, 'scans' if base.endswith('.PROJ') else '', '??????_??????.rxp')
                    
                rxp_files = glob.glob(rxp_pattern)
                if not rxp_files:
                    if self.args.verbose:
                        print(f"!!! Can't find {rxp_pattern} !!!")
                    return
                rxp = rxp_files[0]
                
                matrix_pattern = os.path.join(base, 'matrix', f'{scan.replace(".SCNPOS", "")}.*')
                matrix_files = glob.glob(matrix_pattern)
                if not matrix_files:
                    if self.args.verbose:
                        print(f'!!! Can not find rotation matrix {matrix_pattern} !!!')
                    return
                    
                matrix = np.dot(self.args.global_matrix, np.loadtxt(matrix_files[0]))
                
                if self.args.verbose:
                    with self.args.Lock:
                        print('rxp -> xyz:', rxp)
                
                pipeline = pdal.Pipeline(json.dumps([
                    {"type": "readers.rxp", 
                    "filename": rxp, 
                    "sync_to_pps": "false", 
                    "reflectance_as_intensity": "false"},
                    # "min_reflectance": f"{self.args.reflectance[0]}",
                    # "max_reflectance": f"{self.args.reflectance[1]}"},
                    {"type": "filters.range", 
                    "limits": f"Deviation[0:{self.args.deviation}]"},
                    {"type": "filters.range", 
                    "limits": f"Reflectance[{self.args.reflectance[0]}:{self.args.reflectance[1]}]"},
                    {"type": "filters.transformation", 
                    "matrix": ' '.join(matrix.flatten().astype(str))},
                    {"type": "filters.crop", 
                    "polygon": self.args.convex_hull_wkt},
                    {"type": "filters.splitter",
                    "length": f"{self.args.tile}",
                    "origin_x": f"{math.floor((self.args.bbox[0] - self.args.buffer) / self.args.tile) * self.args.tile}",
                    "origin_y": f"{math.floor((self.args.bbox[2] - self.args.buffer) / self.args.tile) * self.args.tile}"}]))
                
                pipeline.execute()
                
                for arr in pipeline.arrays:
                    if len(arr) == 0:
                        if self.args.verbose:
                            print(f"Skipping empty array for {scan}")
                        continue
                        
                    arr = pd.DataFrame(arr)
                    arr = arr[['X', 'Y', 'Z', 'Reflectance']].rename(
                        columns={'X': 'x', 'Y': 'y', 'Z': 'z', 'Reflectance': 'reflectance'})
                    
                    if len(arr) == 0:
                        continue
                    
                    X, Y = arr[['x', 'y']].min().astype(int)
                    X = math.floor(X / self.args.tile) * self.args.tile
                    Y = math.floor(Y / self.args.tile) * self.args.tile
                    
                    tile = self.args.tiles.loc[(self.args.tiles.x == X) & (self.args.tiles.y == Y)]
                    if len(tile) == 0:
                        continue
                        
                    tile_n = str(tile.tile.item()).zfill(self.args.n)
                    
                    with self.args.Lock:
                        with open(os.path.join(self.args.odir, f'{self.args.plot_code}_{tile_n}.xyz'), 'ab') as fh:
                            fh.write(arr.to_records(index=False).tobytes())
                        
                        with open(os.path.join(self.args.odir, 'tindex.dat'), 'a', newline='') as csvfile:
                            writer = csv.writer(csvfile)
                            writer.writerow([tile.tile.item(), X, Y])
                    
            except Exception as e:
                if self.args.verbose:
                    print(f"Error processing {scan_pos}: {str(e)}")
                return

    def _process_array(self, arr):
        """Process single array output from pipeline"""
        if len(arr) == 0:
            return
            
        arr = arr[['X', 'Y', 'Z', 'Reflectance']].rename(
            columns={'X': 'x', 'Y': 'y', 'Z': 'z', 'Reflectance': 'reflectance'})
        
        X, Y = arr[['x', 'y']].min().astype(int)
        X = math.floor(X / self.args.tile) * self.args.tile
        Y = math.floor(Y / self.args.tile) * self.args.tile
        
        tile = self.args.tiles.loc[(self.args.tiles.x == X) & (self.args.tiles.y == Y)]
        if len(tile) == 0:
            return
            
        tile_n = str(tile.tile.item()).zfill(self.args.n)
        
        with self.args.Lock:
            with open(os.path.join(self.args.odir, f'{self.args.plot_code}_{tile_n}.xyz'), 'ab') as fh:
                fh.write(arr.to_records(index=False).tobytes())
            with open(os.path.join(self.args.odir, 'tindex.dat'), 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([tile.tile.item(), X, Y])

    def process(self):
        """Main processing function"""
        if self.args.print_bbox_only:
            print(self.args.bbox)
            return
            
        with multiprocessing.Manager() as manager:
            self.args.Lock = manager.Lock()
            
            with open(os.path.join(self.args.odir, 'tindex.dat'), 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['tile', 'x', 'y'])
            
            with multiprocessing.Pool(self.args.num_prcs) as pool:
                pool.map(self.process_scan, sorted(self.args.ScanPos))
                xyz_files = glob.glob(os.path.join(self.args.odir, '*.xyz'))
                pool.starmap(self._xyz_to_ply, [(f, self.args) for f in sorted(xyz_files)])
            
            if self.args.plot:
                self._compile_plot()
            
            self._cleanup_tile_index()

    @staticmethod
    def _xyz_to_ply(xyz_path: str, args):
        """Convert XYZ file to PLY format with optional downsampling"""
        if args.verbose:
            with args.Lock:
                print('xyz -> ply:', xyz_path)
        
        with open(xyz_path, encoding='ISO-8859-1') as f:
            data = pd.DataFrame(np.fromfile(f, dtype='float64,float64,float64,float32'))
        
        if args.res > 0:
            downsampler = PointCloudDownsampler(data, args.res)
            idx = downsampler.voxelisation_by_reflectance()
            data = data.iloc[idx]
        
        data.columns = ['x', 'y', 'z', 'reflectance']
        ply_path = xyz_path.replace('.xyz', '.ply')
        plyio.write_ply(ply_path, data)
        os.unlink(xyz_path)

    def _compile_plot(self):
        """Compile PLY files and optionally classify ground"""
        plot_code = self.args.plot_code.replace('_', '')
        outfile = os.path.join(self.args.odir, f'{plot_code}.ply') # Remove old files

        file_pattern = os.path.join(self.args.odir, f'*{plot_code}*.ply')
        ply_files = sorted([file for file in glob.glob(file_pattern) if re.search(r'_\d+\.ply$', os.path.basename(file))])
        print(ply_files)
        exit()
        if ply_files:
            combined = pd.concat([plyio.read_ply(f) for f in ply_files])
            plyio.write_ply(outfile, combined)

            for f in ply_files:
                os.unlink(f)
            
            if self.args.classify_ground:
                self._classify_ground(outfile)
        
        with open(os.path.join(self.args.odir, 'tindex.dat'), 'a', newline='') as f:
            writer = csv.writer(f, delimiter=',')
            writer.writerows([['tile', 'x', 'y'], [0, 15, 15]])

    def _cleanup_tile_index(self):
        """Remove duplicates from tile index"""
        index_path = os.path.join(self.args.odir, 'tindex.dat')
        pd.read_csv(index_path).drop_duplicates().to_csv(index_path, index=False)

def main():
    parser = argparse.ArgumentParser()
    default_procs = max(1, multiprocessing.cpu_count() - 2)
    parser.add_argument('--project', '-p', required=True, type=str, help='path to point cloud')
    parser.add_argument('--plot-code', type=str, help='plot suffix')
    parser.add_argument('--odir', type=str, default='.', help='output directory (default: <project_parent>/clouds)')
    parser.add_argument('--deviation', type=float, default=15, help='deviation filter')
    parser.add_argument('--reflectance', type=float, nargs=2, default=[-999, 999], help='reflectance filter')
    parser.add_argument('--res', type=float, default=0.01, help='voxel resolution to downsample cloud')
    parser.add_argument('--tile', type=float, default=5, help='length of tile')
    parser.add_argument('--plot', action="store_true", help='export as single plot file')
    parser.add_argument('--bbox', type=int, nargs=4, default=[], help='bounding box format xmin xmax ymin ymax')
    parser.add_argument('--num-prcs', type=int, default=default_procs, help='number of cores to use')
    parser.add_argument('--prefix', type=str, default='ScanPos', help='file name prefix')
    parser.add_argument('--buffer', type=float, default=1, help='buffer for bounding box')
    parser.add_argument('--invert', action="store_true", help='invert bounding box')
    parser.add_argument('--global-matrix', type=str, default=False, help='path to global matrix')
    parser.add_argument('--test', action='store_true', help='use .mon.rxp')
    parser.add_argument('--classify-ground', action="store_true", help='Enable ground classification using CSF')
    parser.add_argument('--cloth-resolution', type=float, default=0.10, help='Resolution for cloth simulation in CSF')
    parser.add_argument('--class-threshold', type=float, default=0.10, help='Classification threshold for CSF')
    parser.add_argument('--hdiff', type=float, default=0.10, help='Height difference threshold for CSF')
    parser.add_argument('--rigid', type=float, default=3.0, help='Rigidness of cloth for CSF')
    parser.add_argument('--slope-smooth', action="store_true", default=True, help='Enable slope smoothing in CSF')
    parser.add_argument('--verbose', action='store_true', help='print progress')
    parser.add_argument('--print-bbox-only', action='store_true', help='print bbox only')
    
    processor = PointCloudProcessor(parser.parse_args())
    processor.process()

if __name__ == '__main__':
    main()