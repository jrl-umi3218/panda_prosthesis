#!/usr/bin/bash

# This script is used to generate the convex files from the meshes in ../meshes

# Requires
# - qconvex (sudo apt install qhull-bin)
# - mesh_sampling (https://github.com/jrl-umi3218/mesh_sampling)

readonly this_dir=`cd $(dirname $0); pwd`
readonly meshes_dir=`cd $this_dir/../meshes; pwd`

for m in `find $meshes_dir -type f -name '*.stl'`
do
  qc_out=/tmp/`basename $m ".stl"`.qc
  ch_out=$this_dir/`basename $m ".stl"`-ch.txt
  mesh_sampling $m $qc_out --type xyz --samples 1000 --scale 0.001
  qconvex TI $qc_out TO $ch_out Qt o f
  rm -f $qc_out
done
