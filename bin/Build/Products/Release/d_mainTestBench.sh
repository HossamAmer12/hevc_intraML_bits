
# ./d_mainTestBench.sh YUVs GOPs

# Get Which GOP structure type you will run
# printf "Please enter GOP structure type number: "
# printf " %d" 1
# printf " -> SCD, "
# printf " %d" 2
# printf " -> Even Distributed, "
# printf " %d" 3
# printf " -> Even Dis + A Pre frame\n"
# read -e gop_str_type

# python populateExcel.py populateExcel
#python populateExcel.py 
# read -e bla

# Sequence configuration filename values:
#CFG_FILE1=Keiba
#CFG_FILE2=HallMonitor

# Encoder configuration filename values:
#ENC_CFG_FILE1=encoder_case2
#ENC_CFG_FILE2=encoder_case2

# YUV filename values:
#YUV_FILE1=Keiba_416x240_30
#YUV_FILE2=hall_monitor_CIF

# path_to_disc="/Volumes/MULTICOMHD2/set_yuv"
# path_to_disc="/Volumes/MULTICOM102/103_HA/MULTICOM103/set_yuv"
path_to_disc="/Volumes/MULTICOM105/103_HA/MULTICOM103/set_yuv"

# Get the inputFile for YUVs
inputYUVFileName=$path_to_disc/classes/$1.txt
# for debugging
# inputYUVFileName=$path_to_disc/classes/test.txt

# Get the inputFile for GOPs
inputGOPFileName=Gen/GOP-Types/$2.txt


# Get the inputFile for POCs
inputPOCFileName=Gen/Input-FrameStat/$3.txt

# Get the inputFile for all Bits and PSNR statistics
inputAllBitsPsnrFileName=Gen/Input-FrameStat/$4.txt



# Get the number of sequences
noSeq=$(cat $inputYUVFileName | wc -l )
# echo $noSeq
#echo Please enter the number of Sequences to encode
#read -e noSeq


# Number of Encoder CFG files you would like to run on a sequence
#echo How many encoder CFG files you would like to run on a sequence?
#read -e noCases

# Get the number of cases
noCases=$(cat $inputGOPFileName | wc -l )
# echo $noCases


# Get the number of Qps
echo How many Qp values do you use for encoding?
# read -e noQp
#noQp=4
#noQp=3
#noQp=1
# noQp=2


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



# echo Please enter your Class Number
# read -e classNo

# classNo=$1
# Get the classNumber
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
    # echo ${yuv_file[$i]}


   let "j+=1" # Increment by 2 to get all coulmns
   # echo ${yuv_symbols[$j]}
	
   cfg_file[$i]=${yuv_symbols[j]}
   let "j+=1" # Increment by 2 to get all coulmns
   # echo ${cfg_file[$i]}
   
done


#**********************
# Read YUVs from TXT file
#i=1
#cat "$inputYUVFileName" | while IFS=' ' read first last
#do
    
#    yuv_file[$i]=$first
#    echo ${yuv_file[$i]}

#   cfg_file[$i]=$last
#    echo ${cfg_file[$i]}
#    let "i+=1"
# done


#echo "---"
#printf "\n"
# Input the YUV and CFG files for n sequences
#for (( i = 1; i <= noSeq; i++ ))
#do
#	echo "Enter the YUV filename for Sequence "$i
#	read -e current_yuv
#    yuv_file[$i]=$current_yuv

#	echo "Enter the CFG filename for Sequence "$i
#	read -e current_cfg
#    cfg_file[$i]=$current_cfg
#    echo "---"
#    printf "\n"
# done

#**********************

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
#./d_encodeTestBench.sh $noSeq @ ${yuv_file[@]} @ "${cfg_file[@]}" @ "${Qp[@]}"  \
#                                        @ $noCases @ $noQp # @ $gop_str_type

printf "\n"
echo "Encoder all Done"
echo "#******#******#******#******#******#******#******#******#******#******"

#########

# Agregate files
# for (( i = 1; i <= $noSeq; i++ ))
# do
# 	current_yuv=${yuv_file[$i]}
# 	# concatenate
# 	cat Gen/Seq-Stats/$current_yuv\_{51..0}.txt > Gen/Seq-Stats/$current_yuv.txt

# 	# delete the other files
# 	rm -rf Gen/Seq-Stats/$current_yuv\_*
# done

# echo 'Aggregate files done'

######

./d_decodeMainTestBench.sh $noSeq @ "${yuv_file[@]}" @ "${Qp[@]}"  \
					@ $noCases @ $noQp @ $classNo  
printf "\n"
echo "Decoder all Done"
echo "#******#******#******#******#******#******#******#******#******#******" 					

# Loop on the sequences for every case
# for (( i = 1; i <= noSeq; i++ ))
# do

# 	current_yuv="${yuv_file[$i]}"

# 	#Debug
# 	mkdir Gen/Seq-FramesStat/$current_yuv
# 	# echo $current_yuv

# 	# For every case
# 	for (( caseId = 1; caseId <= noCases; caseId++ ))
# 	do
# 	# Put a seperator between array arguments
# 	./d_frameStat.sh $current_yuv @ $caseId @ "${Qp[@]}"  \
#  					@ $noQp @ $inputPOCFileName

# 	done		
# done

# # Average bits and PSNR of suspected frames
# # Loop on the sequences for every case
# for (( i = 1; i <= noSeq; i++ ))
# do

# 	current_yuv="${yuv_file[$i]}"
# 	echo $current_yuv

# 	#Output file
# 	#current_out=Gen/Seq-FramesStat/Union"_"$current_yuv.txt
# 	current_out=Gen/Seq-FramesStat/Union.txt

# 	echo -e "***********$current_yuv Sequence***********" >> "$current_out"

# 	# For every case
# 	for (( caseId = 1; caseId <= noCases; caseId++ ))
# 	do
# 	# Put a seperator between array arguments
# 	./d_unionframeStat.sh $current_yuv @ $caseId @ "${Qp[@]}"  \
#  					@ $noQp @ $inputPOCFileName

# 	done


# done 

# # Bits Vs PSNR per frame statistics
# # All bits and psnr statistcis
# # Loop on the sequences for every case
# for (( i = 1; i <= noSeq; i++ ))
# do

# 	current_yuv="${yuv_file[$i]}"
# 	echo $current_yuv

# 	#Output file
# 	#current_out=Gen/Seq-FramesStat/Union"_"$current_yuv.txt
# 	current_out=Gen/Seq-FramesStat/UnionBitsPsnr.txt

# 	echo -e "***********$current_yuv Sequence***********" >> "$current_out"

# 	# For every case
# 	for (( caseId = 1; caseId <= noCases; caseId++ ))
# 	do
# 	# Put a seperator between array arguments
# 	./d_bitPsnrframeStat.sh $current_yuv @ $caseId @ "${Qp[@]}"  \
#  					@ $noQp @ $inputAllBitsPsnrFileName

# 	done


# done  

echo "Frames Statistics all Done"
echo "#******#******#******#******#******#******#******#******#******#******"

#TODO: Excel Game starts here --> Use python

#python dynamicExcel.py -n $noSeq -c $noCases \
#				-i in.xls -o out.xls -s "${yuv_file[@]}" \
#				-g IPP,PPI -t summary1,summary2

#python dynamicExcel.py -n $noSeq -c $noCases \
#				-i in2.xls -o out.xls -s "${yuv_file[@]}" \
#				-g "${gop_type_files[@]}" -t "${txt_fileNames[@]}"




echo "Excel all Done"
echo "#******#******#******#******#******#******#******#******#******#******"



# Debug
#for (( i = 1; i <= noSeq; i++ ))
#do
#  echo ${Qp[$i]}
#  echo ${yuv_file[$i]}
#  echo ${cfg_file[$i]}
# done


#******#******#******#******#******#******#******#******#******#******

