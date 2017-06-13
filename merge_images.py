from operator import add
from numpy import subtract
from PIL import Image


# TODO Negative Shift Issue
# Currently, all works for -ve position too
def merge(name_img1, name_img2, position_wrt_img1=(0, 0)):
    if position_wrt_img1 > (0, 0):
        img1 = Image.open(name_img1)
        img2 = Image.open(name_img2)
    else:
        position_wrt_img1 = tuple(subtract((0, 0), position_wrt_img1))
        img2 = Image.open(name_img1)
        img1 = Image.open(name_img2)

    w, h = map(max, map(add, img2.size, position_wrt_img1), img1.size)

    # pasting img1 on img2
    _img1 = Image.new('RGB', size=(w, h), color=0)
    _img1.paste(img1, (0, 0))

    # pasting opposite way
    _img2 = Image.new('RGB', size=(w, h), color=0)
    _img2.paste(img2, position_wrt_img1)

    return Image.blend(_img1, _img2, alpha=0.5)


def merge_BW(name_img1, name_img2, position_wrt_img1=(0, 0)):
    if position_wrt_img1 > (0, 0):
        img1 = Image.open(name_img1)
        img2 = Image.open(name_img2)
    else:
        position_wrt_img1 = tuple(subtract((0, 0), position_wrt_img1))
        img2 = Image.open(name_img1)
        img1 = Image.open(name_img2)

    w, h = map(max, map(add, img2.size, position_wrt_img1), img1.size)

    # pasting img1 on img2
    _img1 = Image.new('L', size=(w, h), color=0)
    _img1.paste(img1, (0, 0))

    # pasting opposite way
    _img2 = Image.new('L', size=(w, h), color=0)
    _img2.paste(img2, position_wrt_img1)

    return Image.blend(_img1, _img2, alpha=0.5)