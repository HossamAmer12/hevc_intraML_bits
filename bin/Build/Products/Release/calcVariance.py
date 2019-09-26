
import glob
import math
import os
import re
import time
import numpy as np

# Fetch the out path
#path_to_union = 'UnionBitsPsnr_org.txt'

path_to_union = 'UnionBitsPsnr_org27.txt'
#path_to_union = 'UnionBitsPsnr_me.txt'

# Fetch the YUV path
path_to_yuv = 'YUVs_sub.txt'

# read YUV file to know the number of sequences
f = open(path_to_yuv, "r")
lines = f.readlines()
f.close()
noSeq=len(lines)


# read the UNION file
f = open(path_to_union, "r")
lines = f.readlines()
f.close()

count_bits=0
count_psnr=0
idx_bits=0
idx_psnr=0

start_bits=0
start_psnr=0

bitsList=[]
psnrList=[]

# read Bits
read_bits_or_psnr=1

# for each sequence calculate the variance
for line_count, line in enumerate(lines):
    if('***********' in line):
#      print 'New Sequence', line
      sequence=line.split('\n')[0]
      count_bits=0
      count_psnr=0
      idx_bits=0
      idx_psnr=0
      bitsList = []
      psnrList = []
      continue

    if('(--*--*-*-*-)' in line):
      continue
    

    if('----------Bits' in line):
        count_bits=0
        start_bits=line_count
        read_bits_or_psnr=1

    if('----------Y-PSNR' in line):
        count_psnr=0
        start_psnr=line_count
        read_bits_or_psnr=0

        # calculate the variance
#        print bitsList
#        print len(bitsList)        
#        print np.var(bitsList)


    # Calculate the PSNR variance
    if ('==========Case' in line):
        if psnrList:
            var_psnr = np.var(psnrList)
            # Write the results
            summary_variance_name = 'variance_report.txt'
            file_id = open(summary_variance_name, 'a')
            out_str=str(len(psnrList)) + '-' + sequence + '\t' + str(var_psnr) + '\n'
            print out_str
            file_id.write(out_str)
            file_id.close()

    elif(read_bits_or_psnr==1 and count_bits == 0 and start_bits != line_count and start_bits > 0):
#        print 'BITS BITS ', line
        str_split = line.split()
        poc=str_split[0]
        bits=float(str_split[1])
        bitsList.append(bits)
        idx_bits = idx_bits + 1

    elif(read_bits_or_psnr==0 and count_psnr == 0 and start_psnr != line_count and start_psnr > 0):
        str_split = line.split()
        poc=str_split[0]
        psnr=float(str_split[1])
        psnrList.append(psnr)
        idx_psnr = idx_psnr + 1


print 'Done with', noSeq, 'sequences.'
