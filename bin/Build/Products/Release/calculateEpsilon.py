
import glob
import math
import os
import re
import time

# Fetch the Videos path
#path_to_videos = '/Users/hossam.amer/7aS7aS_Works/work/workspace/TESTS/SC_t_star_enc/bin/Build/Products/Release/Gen/Seq-TXT_orig_genEps/'
path_to_videos = '/Users/hossam.amer/7aS7aS_Works/work/workspace/TESTS/SC_t_star_enc/bin/Build/Products/Release/Gen/Seq-TXT/'


if path_to_videos[-1] != '/':
    path_to_videos += '/'
    print('Warning: Path to data should end with a slash (/). '
          'Added the slash (/): %s' % path_to_videos)

# Get the MSE files
os.chdir(path_to_videos)
output_MSE_files = glob.glob('*MSE*')
num_output_MSE = len(output_MSE_files)

# Set the Qp length to 4 qps
qpLen = 4
start = -4

# Get the sigma files
os.chdir(path_to_videos)
output_sigma_files = glob.glob('*sigma22*')
num_output_sigma = len(output_sigma_files) # number of sequences

for sequence_count, sigma_file_name in enumerate(output_sigma_files):
    # read the sigma values
    sigma_file = open(sigma_file_name, 'r')
    sigma_line_list = sigma_file.readlines()
    
    # Move the probe to 0
    start = start + qpLen
    
    for mse_file_count in range(start, start + qpLen):
        #for mse_file_name in enumerate(output_MSE_files):
        
       mse_file_name = output_MSE_files[mse_file_count]
        
       probe_str = '(' + str(sequence_count) + ', ' + str(mse_file_count % qpLen) + ') '
       print probe_str, sigma_file_name, ' & ', mse_file_name
       
       # read mse_line
       mse_file = open(mse_file_name, 'r')
       mse_line_list = mse_file.readlines()
       # remove the '\n'
       mse_line_list = [s.replace('\n', '') for s in mse_line_list]
       
       # no_frames
       no_frames = len(mse_line_list)
       # define mu
       mu = [0] * no_frames
       
       # define epsilon and set last epsilon to 1
       epsilon = [0] * no_frames
       epsilon[no_frames-1] = 1
       
      # calculate mu_i
       for i in range(no_frames - 1):
         mu[i] = float(mse_line_list[i+1])/ (float(mse_line_list[i]) + float(sigma_line_list[i+1]))
       # print i, mu[i]
       
       # Calculate epsilon backward except for last frame
       for i in range(no_frames - 2, -1, -1):
        epsilon[i] = 1 + epsilon[i+1] * mu[i]
       # print i, ', eps:', epsilon[i+1], ',mu, ', mu[i], ', eps i', epsilon[i]
       
       # Write the results for mu
       mu_file_name = '_mu_' + str(mse_file_name.split('.txt', 1)[0]) + '.txt'
       mu_file_id = open(mu_file_name, 'a')
       mu_file_id.write(('\n'.join([str(s) for s in mu])))
       mu_file_id.close()

       # Write the results for epsilon
       mu_file_name = '_eps_' + str(mse_file_name.split('.txt', 1)[0]) + '.txt'
       mu_file_id = open(mu_file_name, 'a')
       mu_file_id.write(('\n'.join([str(s) for s in epsilon])))
       mu_file_id.close()



#       break
#    break






