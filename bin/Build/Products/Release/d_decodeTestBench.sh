
#./TAppDecoder -b Gen/test1.265 -o Gen/dec.yuv


# Set the Reconstructed file name
INPUT=$1

# Set the Reconstructed file name
RECONS=$2

# Class Number
classNo=$3

# path_to_disc="/Volumes/MULTICOMHD2/set_yuv"
# path_to_disc="/Volumes/MULTICOM102/103_HA/MULTICOM103/set_yuv"
path_to_disc="/Volumes/MULTICOM105/103_HA/MULTICOM103/set_yuv"

# Run the decoder
./TAppDecoder -b $path_to_disc/Seq-265/$classNo/$INPUT.265 -o $path_to_disc/Seq-RECONS/$classNo/$RECONS.yuv



