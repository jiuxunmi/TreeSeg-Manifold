#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <data_prefix> <\"coords\">"
    echo "Example: $0 data/sample_plot \"-4 17 -16 5\""
    exit 1
fi

PREFIX=$1
COORDS=$2
COORD_FILE="${PREFIX}.coords.dat"
BASENAME=$(basename "$PREFIX")

echo ${COORDS} > ${COORD_FILE}

BIN_DIR="./build"

if [ ! -d "$BIN_DIR" ]; then
    mkdir -p build
    cd build
    cmake ..
    make -j4
    cd ..
fi

${BIN_DIR}/downsample 0.04 ${PREFIX}.tile.0.pcd
${BIN_DIR}/getdtmslice 2 2.5 1 4 ${PREFIX}.tile.downsample.0.pcd > ${PREFIX}.dtm.dat

mkdir -p clusters
cd clusters
../${BIN_DIR}/findstems 4 0.2 2 ../${COORD_FILE} ../${PREFIX}.slice.pcd
cd ..

mkdir -p stems
cd stems
../${BIN_DIR}/segmentstem 4 ../clusters/${BASENAME}.cluster.*.pcd ../${PREFIX}.tile.downsample.0.pcd
cd ..

${BIN_DIR}/step1_supervoxelizer 0.05 0.20 ${PREFIX}.tile.downsample.0.pcd
${BIN_DIR}/step2_graph_grower stems/${BASENAME}.stem.*.pcd