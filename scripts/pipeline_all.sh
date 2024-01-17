#!/bin/bash

# PARAMETERS
# -e        ENVIRONMENT NAME
# -v        VIEW RESULTS ?
# -s        STEP []
# -c        CONTINUE []

# SCRIPT PATHS

CAPTURE_PYTHON_SCRIPT="$HOME/dev/AdaptiveGoalRegion/src/capture.py"
TRANSFORMATION_PYTHON_SCRIPT="$HOME/dev/AdaptiveGoalRegion/src/transform_and_cluster.py"
CONDA_PATH=$HOME/miniconda3/etc/profile.d/conda.sh

# Default values
env_name=""
view=false
step=0

# Parse arguments
while getopts "e:vs:" opt; do
  case $opt in
    e) env_name="$OPTARG"
       ;;
    v) view=true
       ;;
    s) step="$OPTARG"
       ;;
    \?) echo "Invalid option -$OPTARG" >&2
        exit 1
       ;;
  esac
done

if [ -z "$env_name" ]; then
    echo "Environment name (-e) is required."
    exit 1
fi

# --------------------------

# STEP 001
#
# ARRANGE DIRECTORY FOR ENVIRONMENT
#

# Define the target directory
target_dir="$HOME/dev/AdaptiveGoalRegion/storage/$env_name"
export PYTHONPATH=$PYTHONPATH:"$HOME/dev/AdaptiveGoalRegion"
export PYTHONPATH=$PYTHONPATH:"$HOME/dev/uois"

# Create environment directory if it does not exist
if [ ! -d "$target_dir" ]; then
    mkdir -p "$target_dir"
fi

# Rename "temp" directory and create a new one
if [ -d "$target_dir/temp" ]; then
    # Find the last number in the directory
    last_num=$(ls -v "$target_dir" | grep -E '^[0-9]+$' | tail -n 1)
    next_num=$((last_num + 1))

    # Rename "temp" to the next number
    mv "$target_dir/temp" "$target_dir/$next_num"
fi

# Create a new "temp" directory
mkdir "$target_dir/temp"

# Output based on view flag
if $view; then
    echo "Environment: $env_name"
    echo "Step: $step"
    ls -l "$target_dir"
fi


# STEP 002
#
# CAPTURE IMAGE WITH GAZEBO & FILTER [~AdaptiveGoalRegion] -> capture.py [raw_capture.py]
#

python3 "$CAPTURE_PYTHON_SCRIPT" -e "$env_name"

# STEP 003
#
# SEGMENTATION (RETURN FILTERED DEPTH) [~CONDA/UOIS] -> main [seg_data.np]
#

source $CONDA_PATH
conda activate uois_env
echo $PYTHONPATH
cd $HOME/dev/uois
export PYTHONPATH=$PYTHONPATH:$(pwd)

python $HOME/dev/uois/process.py -e "$env_name" -i "$target_dir/temp"

# STEP 004
#
# CONTACT GRASP-NET [~CONDA/CONTACT_GRASPNET_ENV] -> main [poses.npz]
#

source $CONDA_PATH
conda activate contact_graspnet_env
cd $HOME/dev/contact_graspnet/
export PYTHONPATH=$PYTHONPATH:$(pwd)
python contact_graspnet/inference.py --np_path=$target_dir/temp/seg_data.npy --forward_passes=5 --filter_grasps --z_range=[0.2,1.4] --output_dir="$target_dir/temp/"
conda deactivate

# STEP 005
#
# TRANSFORMATION AND CLUSTERING [~AdaptiveGoalRegion] -> src/transformation_and_clustering.py [agr_output_...txt]
#
export PYTHONPATH=$PYTHONPATH:$(pwd)
/usr/bin/python3.8 "$TRANSFORMATION_PYTHON_SCRIPT" -i "$target_dir/temp/poses.npz" -o "$target_dir/temp/"

# STEP 006
#
# OVERWRITE storage/spline_finals FOR OUTPUTS


exit

