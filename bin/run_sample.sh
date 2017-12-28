#env for running samples
export VOXEL_SDK_PATH=(path to this sdk directory)
export LD_LIBRARY_PATH=$VOXEL_SDK_PATH/lib:$LD_LIBRARY_PATH
export PATH=$VOXEL_SDK_PATH/lib:$VOXEL_SDK_PATH/bin:$PATH
./HaloH1Sample
