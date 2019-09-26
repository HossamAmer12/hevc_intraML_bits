# path_to_disc="/Volumes/MULTICOM102/103_HA/MULTICOM103/set_yuv"
path_to_disc="/Volumes/MULTICOM105/103_HA/MULTICOM103/set_yuv"

inputYUVFileName=$path_to_disc/classes/$1.txt
inputGOPFileName=Gen/GOP-Types/$2.txt
inputPOCFileName=Gen/Input-FrameStat/$3.txt
inputAllBitsPsnrFileName=Gen/Input-FrameStat/$4.txt
noSeq=$(cat $inputYUVFileName | wc -l )

# noSeq=3

noCases=$(cat $inputGOPFileName | wc -l )
# echo $noCases


# Get the number of Qps
echo How many Qp values do you use for encoding?
# read -e noQp
#noQp=4
#noQp=3
#noQp=1
# noQp=1


noQp=26

# Get the first Qp
echo Please enter the start of the Qp range
# read -e startQp
#startQp=22
#startQp=0
#startQp=2
startQp=0

# Get the Qp step size
echo Please enter your Qp step size
# read -e stepSizeQp
stepSizeQp=2

classNo=$(echo $1 | cut -d'_' -f 2)



# echo “Reminder: This bash file takes a parameter which case”

# http://unix.stackexchange.com/questions/88100/importing-data-from-a-text-file-to-a-bash-script
declare -a yuv_symbols=($(cat $inputYUVFileName| tr '\n' ' '))

# Read YUVs from TXT file
i=1
j=0
#loopCount=$(($noSeq*2))

for (( i = 1; i <= noSeq; i++ ))
do
	# echo ${yuv_symbols[$j]}
	
	yuv_file[$i]=${yuv_symbols[j]}
    echo ${yuv_file[$i]}
    # exit


   let "j+=1" # Increment by 2 to get all coulmns
   # echo ${yuv_symbols[$j]}
	
   cfg_file[$i]=${yuv_symbols[j]}
   let "j+=1" # Increment by 2 to get all coulmns
   # echo ${cfg_file[$i]}
   
done



# To make it start with startQp
startQp=50

Qp[1]=51

scaleQpIndex=0
#Create the Qp range
for ((i = 2; i<=noQp;i++))
do
  # Qp[$i]=$(($startQp + $scaleQpIndex*$stepSizeQp))
  Qp[$i]=$(($startQp - $scaleQpIndex*$stepSizeQp))
  let "scaleQpIndex+=1"

done  

### Add two other QPs:
idx=$((noQp+1))
# idx2=$((idx+1))
Qp[$idx]=0
# Qp[$idx2]=51
noQp=$(($noQp+1))
echo 'Updated the total number of Qps: ' $noQp
printf '%s\n' "${Qp[@]}"
# exit

#**********************

# Get the GOP structure names
declare -a gop_type_files=($(cat $inputGOPFileName| tr '\n' ' '))

txt_fileNamesIndex=1
# Parse to get the txt_fileNames
for (( i = 1; i <= noSeq; i++ ))
do

	current_yuv="${yuv_file[$i]}"

	# For every case
	for (( caseId = 1; caseId <= noCases; caseId++ ))
	do
		# For every Qp value
		for (( qpId = 1; qpId <= noQp; qpId++ ))
		do
		
			txt_fileNames[txt_fileNamesIndex]=$current_yuv"_"${Qp[$qpId]}"_"$caseId
			let "txt_fileNamesIndex+=1" 
		done
	done		
done


#**********************

# Put a seperator between array arguments
./d_encodeTestBenchParallel.sh $noSeq @ ${yuv_file[@]} @ "${cfg_file[@]}" @ "${Qp[@]}"  \
 					@ $noCases @ $noQp @ $classNo

printf "\n"
echo "Encoder all Done"
echo "#******#******#******#******#******#******#******#******#******#******"

./d_decodeMainTestBench.sh $noSeq @ "${yuv_file[@]}" @ "${Qp[@]}"  \
					@ $noCases @ $noQp @ $classNo  
printf "\n"
echo "Decoder all Done"
echo "#******#******#******#******#******#******#******#******#******#******" 	

