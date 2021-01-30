# Apriltag generator, from iosoft.blog
# Generates Apriltags for testing
# https://iosoft.blog/2019/09/02/raspberry-pi-position-detection-fiducial-tags/

# Usage
# python3 apriltag_generator.py [-f tagfamily] [filename]

# Example
# python3 apriltag_generator.py -f tag16h5 test.jpg

import sys, math, numpy as np, svgwrite
from PIL import Image
 
filename  = 'test.svg'  # Default filename (.svg, .png, .jpeg or .pgm)
family    = 'tag16h5'   # Default tag family (see tag_families)
NTAGS     = 10          # Number of tags to create
TAG_PITCH = 10          # Spacing of tags
WHITE     = 255         # White colour (0 is black)
 
# First 10 values of 3 tag families
tag16h5 =  16, 5,(0x231b,0x2ea5,0x346a,0x45b9,0x79a6,
                  0x7f6b,0xb358,0xe745,0xfe59,0x156d)
tag25h9  = 25, 9,(0x155cbf1,0x1e4d1b6,0x17b0b68,0x1eac9cd,0x12e14ce,
                  0x3548bb,0x7757e6,0x1065dab,0x1baa2e7,0xdea688)
tag36h11 = 36,11,(0xd5d628584,0xd97f18b49,0xdd280910e,0xe479e9c98,0xebcbca822,
                  0xf31dab3ac,0x056a5d085,0x10652e1d4,0x22b1dfead,0x265ad0472)
tag_families = {"tag16h5":tag16h5, "tag25h9":tag25h9, "tag36h11":tag36h11}
 
# Set up the graphics file, given filename and tag family
def set_graphics(fname, family):
    global FTYPE, IMG_WD, IMG_HT, SCALE, DWG_SIZE, VIEW_BOX
    FTYPE = fname.split('.')[-1].upper()
    FTYPE = FTYPE.replace("PGM", "PPM").replace("JPG", "JPEG")
    IMG_HT = int(math.sqrt(family[0])) + 6
    IMG_WD = (NTAGS-1)*TAG_PITCH + IMG_HT
 
    # Vector definitions
    if FTYPE == "SVG":
        SCALE     = 2
        DWG_SIZE  = "%umm"%(IMG_WD*SCALE),"%umm"%(IMG_HT*SCALE)
        VIEW_BOX  = "0 0 %u %s" % (IMG_WD, IMG_HT)
 
    # Bitmap definitions
    else:
        SCALE = 10
 
# Generate a tag with the given value, return a numpy array
def gen_tag(tag, val):
    area, minham, codes = tag
    dim = int(math.sqrt(area))
    d = np.frombuffer(np.array(codes[val], ">i8"), np.uint8)
    bits = np.unpackbits(d)[-area:].reshape((-1,dim))
    bits = np.pad(bits, 1, 'constant', constant_values=0)
    return np.pad(bits, 2, 'constant', constant_values=1)
 
# Save numpy arrays as a bitmap
def save_bitmap(fname, arrays):
    img = Image.new('L', (IMG_WD,IMG_HT), WHITE)
    for i,a in enumerate(arrays):
        t = Image.fromarray(a * WHITE)
        img.paste(t, (i*TAG_PITCH,0))
    img = img.resize((IMG_WD*SCALE, IMG_HT*SCALE))
    img.save(fname, FTYPE)
 
# Save numpy arrays as a vector file
def save_vector(fname, arrays):
    dwg = svgwrite.Drawing(fname, DWG_SIZE, viewBox=VIEW_BOX, debug=False)
    for i,a in enumerate(arrays):
        g = dwg.g(stroke='none', fill='black')
        for dy,dx in np.column_stack(np.where(a == 0)):
            g.add(dwg.rect((i*TAG_PITCH + dx, dy), (1, 1)))
        dwg.add(g)
    dwg.save(pretty=True)
 
if __name__ == '__main__':
    opt = None
    for arg in sys.argv[1:]:    # Process command-line arguments..
        if arg[0]=="-":
            opt = arg.lower()
        else:
            if opt == '-f':     # '-f family': tag family
                family = arg
            else: 
                filename = arg  # 'filename': graphics file  
            opt = None
    if family not in tag_families:
        print("Unknown tag family: '%s'" % family)
        sys.exit(1)
    tagdata = tag_families[family]
    set_graphics(filename, tagdata)
    print("Creating %s, file %s" % (family, filename))
    tags = [gen_tag(tagdata, n) for n in range(0, NTAGS)]
    if FTYPE == "SVG":
        save_vector(filename, tags)
    else:
        save_bitmap(filename, tags)
