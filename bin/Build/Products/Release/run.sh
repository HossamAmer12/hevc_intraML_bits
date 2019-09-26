# cd /Users/hossam.amer/7aS7aS_Works/work/workspace/TESTS/SC_RB_FullTest-3T/bin/Build/Products/Release/
# ./d_mainTestBench.sh YUVsT1 GOPs POCs AllBitsPsnr


n1=1
n2=11

for (( c=$n1; c<=$n2; c++ ))
do  
    date;time ./d_mainTestBench.sh class_$c GOPs POCs AllBitsPsnr
    printf "\n"
    echo "#******#******#******#******#******#******#******#******#******#******"
	echo "Class-"$c " Done."
	echo "#******#******#******#******#******#******#******#******#******#******"
done
