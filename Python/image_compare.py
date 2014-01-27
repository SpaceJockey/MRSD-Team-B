import pylab as P
import numpy as N

# read the images    
im1 = P.imread('4Fsjx.jpg')
im2 = P.imread('xUHhB.jpg')

# do the crosscorrelation
conv = N.convolve(im1, im2)
# a measure for similarity then is:
sim = N.sum(N.flatten(conv))