# import the necessary packages
from skimage import measure
import matplotlib.pyplot as plt
import numpy as np
import cv2
from video_capture import VideoCaptureYUV
import math


import numpy as np
import tensorflow as tf
from skimage import data, img_as_float

import tensorflow as tf
import numpy as np

def mse(imageA, imageB):
	# the 'Mean Squared Error' between the two images is the
	# sum of the squared difference between the two images;
	# NOTE: the two images must have the same dimension
	err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
	err /= float(imageA.shape[0] * imageA.shape[1])

	# return the MSE, the lower the error, the more "similar"
	# the two images are
	return err


def compare_images(imageA, imageB, title):
	# compute the mean squared error and structural similarity
	# index for the images
	m = mse(imageA, imageB)
	s = measure.compare_ssim(imageA, imageB)
 
	# setup the figure
	fig = plt.figure(title)
	plt.suptitle("MSE: %.2f, SSIM: %.2f" % (m, s))
 
	# show first image
	ax = fig.add_subplot(1, 2, 1)
	plt.imshow(imageA, cmap = plt.cm.gray)
	plt.axis("off")
 
	# show the second image
	ax = fig.add_subplot(1, 2, 2)
	plt.imshow(imageB, cmap = plt.cm.gray)
	plt.axis("off")
 
	# show the images
	plt.show()



height = 240
width  = 416
image = "/Volumes/DATA/TS/BQSquare_416x240_60.yuv"
size = (height, width) # height and then width
videoObj = VideoCaptureYUV(image, size, isGrayScale=False)
ret, yuv, rgb = videoObj.getYUVAndRGB()


image = "/Users/hossam.amer/7aS7aS_Works/work/workspace/TESTS/hevc_intraML_bits/bin/Build/Products/Release/Gen/Seq-RECONS/BQSquare_416x240_60_22_1.yuv"
size = (height, width) # height and then width
videoObj = VideoCaptureYUV(image, size, isGrayScale=False)
ret, yuvEncoded, rgb = videoObj.getYUVAndRGB()


# Using sckit-image
print('------')
y_ssim = measure.compare_ssim(yuvEncoded[:, :, 0],  yuv[:, :, 0], multichannel=True)
cb_ssim = measure.compare_ssim(yuvEncoded[:, :, 1], yuv[:, :, 1], multichannel=True)
cr_ssim = measure.compare_ssim(yuvEncoded[:, :, 2], yuv[:, :, 2], multichannel=True)
print('Y-SSIM: ', y_ssim)
final_ssim = (6.0 * y_ssim + cb_ssim + cr_ssim) / 8.0
print(final_ssim)
