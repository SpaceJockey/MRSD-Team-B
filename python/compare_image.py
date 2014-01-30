import math, operator
import image
import ImageChops

def compare(file1, file2):
    image1 = Image.open(file1)
    image2 = Image.open(file2)
    h1 = image1.histogram()
    h2 = image2.histogram()
    rms = math.sqrt(reduce(operator.add,
                           map(lambda a,b: (a-b)**2, h1, h2))/len(h1))
    return rms



def equal(im1, im2):
    return ImageChops.difference(im1, im2).getbbox() is None

if __name__=='__main__':
    import sys
    file1, file2 = sys.argv[1:]
    print equal(file1, file2)

    #sasha prabu , vaibhav saxena

