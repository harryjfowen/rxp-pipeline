#!/bin/bash

# Configuration variables
PDAL_VERSION="2.5.5"
CONDA_ENV_NAME="pdal"
RIVLIB_VERSION="2_8_0"
RDBLIB_VERSION="2.4.5"
GCC_VERSION="gcc9"
WORK_DIR="$HOME/pdal_build"
DOWNLOADS_DIR="$HOME/Downloads"

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to print status messages
print_status() {
    echo "===> $1"
}

# Check for required tools
if ! command_exists conda; then
    echo "Error: conda is not installed. Please install Miniconda or Anaconda first."
    exit 1
fi

if ! command_exists wget; then
    echo "Error: wget is not installed. Please install it first."
    exit 1
fi

# Create working directory
mkdir -p "$WORK_DIR"
cd "$WORK_DIR"

# Download and extract PDAL source
print_status "Downloading PDAL source..."
wget "https://github.com/PDAL/PDAL/releases/download/$PDAL_VERSION/PDAL-$PDAL_VERSION-src.tar.bz2"
tar -xf "PDAL-$PDAL_VERSION-src.tar.bz2"

# Check for RIEGL libraries in the Downloads folder and move them to the work directory
if [ -f "$DOWNLOADS_DIR/rivlib-${RIVLIB_VERSION}-x86_64-linux-${GCC_VERSION}.zip" ] && [ -f "$DOWNLOADS_DIR/rdblib-${RDBLIB_VERSION}-x86_64-linux.tar.gz" ]; then
    cp "$DOWNLOADS_DIR/rivlib-${RIVLIB_VERSION}-x86_64-linux-${GCC_VERSION}.zip" "$WORK_DIR/"
    cp  "$DOWNLOADS_DIR/rdblib-${RDBLIB_VERSION}-x86_64-linux.tar.gz" "$WORK_DIR/"
fi

# Check for RIEGL libraries in the work directory
if [ ! -f "$WORK_DIR/rivlib-${RIVLIB_VERSION}-x86_64-linux-${GCC_VERSION}.zip" ] || [ ! -f "$WORK_DIR/rdblib-${RDBLIB_VERSION}-x86_64-linux.tar.gz" ]; then
    echo "Please download the following files from RIEGL member area and place them in $WORK_DIR or $DOWNLOADS_DIR:"
    echo "1. rivlib-${RIVLIB_VERSION}-x86_64-linux-${GCC_VERSION}.zip"
    echo "2. rdblib-${RDBLIB_VERSION}-x86_64-linux.tar.gz"
    exit 1
fi

# Create conda environment
print_status "Creating conda environment..."
conda create -n $CONDA_ENV_NAME -c conda-forge -y gdal ninja cmake cxx-compiler laszip pdal python-pdal pandas geopandas

# Activate conda environment
print_status "Activating conda environment..."
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate $CONDA_ENV_NAME

# Extract RIEGL libraries
print_status "Extracting RIEGL libraries..."
unzip -q "rivlib-${RIVLIB_VERSION}-x86_64-linux-${GCC_VERSION}.zip"
tar -xf "rdblib-${RDBLIB_VERSION}-x86_64-linux.tar.gz"

# Set environment variables
export RiVLib_DIR="$WORK_DIR/rivlib-${RIVLIB_VERSION}-x86_64-linux-${GCC_VERSION}"
export rdb_DIR="$WORK_DIR/rdblib-${RDBLIB_VERSION}-x86_64-linux/interface/cpp"

# Modify PDAL cmake files
print_status "Configuring PDAL build..."
cd "PDAL-$PDAL_VERSION-src"
sed -i 's/"Choose if RiVLib support should be built" FALSE)/"Choose if RiVLib support should be built" True)/g' cmake/options.cmake
sed -i 's/"Choose if rdblib support should be built" FALSE)/"Choose if rdblib support should be built" True)/g' cmake/options.cmake
sed -i 's/const bool DEFAULT_SYNC_TO_PPS = true;/const bool DEFAULT_SYNC_TO_PPS = false;/g' plugins/rxp/io/RxpReader.hpp

# Build PDAL
print_status "Building PDAL..."
mkdir -p build
cd build

#Fix PDAL bug 
echo "Please open the XMLSchema.cpp and XMLSchema.hpp file and do the following:"
echo "Change any 'xmlErrorPtr error' to 'xmlError const*error'
echo "Once done, press Enter to continue with the build process."
read -p "Press Enter when you have made the change..."
echo "Proceeding with cmake..."

#Continue building PDAL
cmake -G Ninja ..
ninja

# Install plugins
print_status "Installing PDAL plugins..."
CONDA_LIB_PATH="$(conda info --base)/envs/$CONDA_ENV_NAME/lib"
cp lib/libpdal_plugin_reader_*.so "$CONDA_LIB_PATH/"

# Add PDAL to PATH
PDAL_BIN="$WORK_DIR/PDAL-$PDAL_VERSION-src/build/bin"
echo "export PATH=$PDAL_BIN:\$PATH" >> ~/.bashrc
export PATH="$PDAL_BIN:$PATH"

print_status "Installation complete!"
echo "Please run 'source ~/.bashrc' to update your PATH"
echo "To use PDAL, activate the conda environment with: conda activate $CONDA_ENV_NAME"

# Verify installation
print_status "Verifying installation..."
pdal --version

if [ $? -eq 0 ]; then
    echo "PDAL installation successful!"
else
    echo "Warning: PDAL installation may have issues. Please check the logs above."
fi
