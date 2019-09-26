
# Write it with spaces! -- No Escape characters and don't add "/" at the end
#sequences_dir=$HOME/"7aS7aS_Works/work/workspace/TS"
# echo "TSssss: "$sequences_dir

# for YouTube
# sequences_dir="/Volumes/DATA/TS"

# sequences_dir="/Volumes/DATA/TS"

# for ML
# path_to_disc="/Volumes/MULTICOMHD2/set_yuv"
# path_to_disc="/Volumes/MULTICOM102/103_HA/MULTICOM103/set_yuv"
path_to_disc="/Volumes/MULTICOM105/103_HA/MULTICOM103/set_yuv"
sequences_dir=$path_to_disc/pics

# Number of Sequences
noSeq=$1 #Send it as the first var we raya7 demaa3'ak

index=1          # Reset count.
# Index of the given argument by the top shell file
argIndex=0


# Index of the CFG array
qpIndex=1 #Start from 1 because you already started from 1


# For reference: ./d_maxBoundRun.sh (0) $noSeq @ (1) $current_yuv 
# 									@ (2) $current_cfg @ (3) "${Qp[@]}" 
#									@ (4)$noCases @ (5)noQp

# Parse the arguments to get the array of YUVs, Sequence CFGs
for arg in "$@"
do

# Found a seperator --> Found a new arg
if [ "$arg" == "@" ]  ; then  
	let "argIndex+=1"
fi

# Filter the 2nd argument (YUV_Array) sent and populate it in the list
if [ "$argIndex" == 1 ] && [ "$arg" != "@" ]  ; then  
  	current_yuv=$arg

  	#echo $current_yuv
fi

# Filter the 3rd argument (CFG_Array) sent and populate it in the list
if [ "$argIndex" == 2 ] && [ "$arg" != "@" ]  ; then  
  current_cfg=$arg

 # echo $current_cfg
fi

# Filter the 3rd argument (Qp_Array) sent and populate it in the list
if [ "$argIndex" == 3 ] && [ "$arg" != "@" ]  ; then  
  Qp[$qpIndex]=$arg

#  echo ${Qp[$qpIndex]}
  let "qpIndex+=1"
fi

# Get the number of cases #TODO: Instead of taking no of cases, take the curent enc cfg
if [ "$argIndex" == 4 ] && [ "$arg" != "@" ]  ; then  
 	noCases=$arg
  # echo $noCases
fi

# Get the number of Qp values
if [ "$argIndex" == 5 ] && [ "$arg" != "@" ]  ; then  
  noQp=$arg

   # echo $noQp
fi

# Get the GOP structure type
if [ "$argIndex" == 6 ] && [ "$arg" != "@" ]  ; then  
  gop_str_type=$arg
 # echo $gop_str_type
fi

# Get the GOP structure type
if [ "$argIndex" == 7 ] && [ "$arg" != "@" ]  ; then  
  current_enc_cfg=$arg
  # echo "Current Enc CFG Received: " $current_enc_cfg
  # read -e hohoz
fi

# Get the classNumber 
if [ "$argIndex" == 8 ] && [ "$arg" != "@" ]  ; then  
  classNo=$arg
  #echo "Current Enc CFG Received: " $classNo
  #read -e hohoz
fi

done             # $@ sees arguments as separate words. 

#-- not test >> + Gop try type only + bug in Qp fixed $

sequences_dir=$sequences_dir/$classNo
# echo $sequences_dir
# read -e hello hello


echo BS Output has this format YUV_FILE_QP_GOP_STR_TYPE
echo TXT Output has this format TXTnum_YUV_FILE_QP_GOP_STR_TYPE

# Hardcode
# For every YUV, CFG, Case sequence file -> Run All Qps
for ((i = 1; i<=noQp;i++))
#Hardcode
#for ((i = 0; i<4;i++))
do
	printf "\n"
	echo "Start Qp value" ${Qp[$i]} "Run for" $current_yuv "sequence"
	# Set the BS file output name for 5 Qps
	# BS1=$YUV_FILE"_"${Qp[$i]}"_"$GOP_STR_TYPE
	 current_bs=$current_yuv"_"${Qp[$i]}"_"$gop_str_type

   # Set the BS file output name for 5 Qps
   #TXT1=$YUV_FILE"_"$Qp1"_"$GOP_STR_TYPE
   current_txt=$current_yuv"_"${Qp[$i]}"_"$gop_str_type
   
	# echo "BS"$i $current_bs
	# echo "TXT"$i $current_txt
	# echo "GOP"$i $gop_str_type

   # Run i:
# ./TAppEncoder -c cfg/$current_enc_cfg.cfg -c cfg/per-sequence/$current_cfg.cfg  \
#              -i "$sequences_dir"/$current_yuv.yuv -q ${Qp[$i]}  \
#             -b Gen/Seq-265/$current_bs.265 > Gen/Seq-TXT/$current_txt.txt

# Run all -- April 22, 2019
# ./TAppEncoder -c cfg/$current_enc_cfg.cfg -c cfg/per-sequence/$current_cfg.cfg -i "$sequences_dir"/$current_yuv.yuv -q ${Qp[$i]} -b Gen/Seq-265/$current_bs.265 > Gen/Seq-TXT/$current_txt.txt


# ML HEVC Project:
test_yuv=${current_yuv##*_}
# echo $test_yuv

if [ "$test_yuv" == "Y" ] ; then
  ./TAppEncoder -c cfg/encoder_1Y.cfg  -c $path_to_disc/cfgs/$classNo/$current_cfg.cfg  \
        -i "$sequences_dir"/$current_yuv.yuv -q ${Qp[$i]} -f 1 \
        -b $path_to_disc/Seq-265/$classNo/$current_bs.265 > $path_to_disc/Seq-TXT/$classNo/$current_txt.txt
else
  ./TAppEncoder -c cfg/$current_enc_cfg.cfg  -c $path_to_disc/cfgs/$classNo/$current_cfg.cfg  \
        -i "$sequences_dir"/$current_yuv.yuv -q ${Qp[$i]} -f 1 \
        -b $path_to_disc/Seq-265/$classNo/$current_bs.265 > $path_to_disc/Seq-TXT/$classNo/$current_txt.txt
fi


echo "Done Qp value" ${Qp[$i]} "Run for" $current_yuv "sequence"
echo "@@@@@@@@@@@@@@@@"

done

printf "\n"
echo "Done with" $noQp "Runs for" $current_yuv "sequence"
# echo "***********************"
# printf "\n"

# DEBUG BreakPoint
# read -e bla_bla



# Rename the summary files to avoid overwriting
# mv summaryTotal.txt "SumTot_"$current_yuv"_"$gop_str_type.txt
# mv summary_I.txt "SumI_"$current_yuv"_"$gop_str_type.txt
# mv summary_P.txt "SumP_"$current_yuv"_"$gop_str_type.txt
# mv summary_B.txt "SumB_"$current_yuv"_"$gop_str_type.txt

# Get the Sequence config filename
