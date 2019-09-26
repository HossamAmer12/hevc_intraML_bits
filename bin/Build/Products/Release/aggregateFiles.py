import math
import glob

import os
import numpy as np 

import matplotlib.pyplot as plt

# start image
start = 1
# end image
end = 1 + 1000


path_to_txt_files ='Gen/Seq-Stats/'
path_out_txt_files = 'Gen/Seq-Stats-Unified/'

def readFileContents(image):
	f = open(image, "r")	
	lines = f.readlines()
	f.close()
	return lines

def writeFileContents(image, lines):
	f = open(image, "a")
	f.write(''.join(lines))
	f.close()


QP = []
QP.append(51)
for i in range(50, 0, -2):
	QP.append(i)
QP.append(0)

print('QPs: ', QP)
print('Number of QPs: ', len(QP))


# Create a directory
try:
    # Create target Directory
    os.mkdir(path_out_txt_files)
    print("Directory " , path_out_txt_files ,  " created. ") 
except FileExistsError:
    print("Directory " , path_out_txt_files ,  " already exists.")

# Create bpp ORG, SSIM Org, PSNR ORG lists.
for imgID in range(start, end):

	original_img_ID = imgID
	imgID = str(imgID).zfill(8)
	shard_num  = round(original_img_ID/10000);
	folder_num = math.ceil(original_img_ID/1000);
	filesList = glob.glob(path_to_txt_files + 'ILSVRC2012_val_' + imgID + '*.txt')	

	#ILSVRC2012_val_00000001_504_376_RGB_10
	name = filesList[0].split('/')[-1]
	qp = name.split('_')[-1].split('.')[0]
	rgbStr = name.split('_')[-2]
	height = int(name.split('_')[-3])
	width  = int(name.split('_')[-4])
	

	desired_image = path_out_txt_files + 'ILSVRC2012_val_' + imgID + '_' + str(width) + '_' + str(height) + '_' + rgbStr + '.txt'

	for iqp, qp in enumerate(QP):
		current_image = path_to_txt_files + 'ILSVRC2012_val_' + imgID + '_' + str(width) + '_' + str(height) + '_' + rgbStr + '_' + str(qp) + '.txt'
		lines = readFileContents(current_image)
		#writeFileContents(desired_image, lines)
		#os.remove(current_image)
		print('Remove: ', current_image)
		#print('Write', desired_image)
	
	if  not original_img_ID % 1000:
		print('Aggregate done with %s images.' % imgID)
	



	