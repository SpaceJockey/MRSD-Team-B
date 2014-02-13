import pylab as P
import numpy as N

# read the images    
im1 = P.imread('ger1.png')
im2 = P.imread('ger2.png')

# do the crosscorrelation
conv = N.convolve(im1, im2)
# a measure for similarity then is:
sim = N.sum(N.flatten(conv))