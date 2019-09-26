# cd /Users/hossam.amer/7aS7aS_Works/work/workspace/TESTS/SC_RB_FullTest-3T/bin/Build/Products/Release/
# ./d_mainTestBench.sh YUVsT1 GOPs POCs AllBitsPsnr

function runCode {

n1=$1
n2=$2

for (( c=$n1; c<=$n2; c++ ))
do  
    date;time ./d_mainTestBench.sh class_$c GOPs POCs AllBitsPsnr
    printf "\n"
    echo "#******#******#******#******#******#******#******#******#******#******"
        echo "Class-"$c " Done."
        echo "#******#******#******#******#******#******#******#******#******#******"
done

}


# Class1: Done
# runCode 1 11
# Class 2: Done
#runCode 12 21
# Class 3: Done
runCode 22 31
# Class 4: Done
#runCode 32 41
# Class 5: 
# runCode 42 51

#n1=1
#n2=11

#for (( c=$n1; c<=$n2; c++ ))
#do  
#    date;time ./d_mainTestBench.sh class_$c GOPs POCs AllBitsPsnr
#    printf "\n"
#    echo "#******#******#******#******#******#******#******#******#******#******"
#	echo "Class-"$c " Done."
#	echo "#******#******#******#******#******#******#******#******#******#******"
#done
