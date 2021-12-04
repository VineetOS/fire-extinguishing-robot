import os.path
import numpy as np
from PIL import Image


def pil2numpy(cv_img) -> np.ndarray:
    """
    Convert an HxW pixels RGB Image into an HxWx3 numpy ndarray
    """

    # if img is None:
    #     img = Image.open(cv_img)

    np_array = np.asarray(cv_img)
    return np_array


def numpy2pil(np_array: np.ndarray) -> Image:
    """
    Convert an HxWx3 numpy array into an RGB Image
    """

    assert_msg = 'Input shall be a HxWx3 ndarray'
    assert isinstance(np_array, np.ndarray), assert_msg
    assert len(np_array.shape) == 3, assert_msg
    assert np_array.shape[2] == 3, assert_msg

    img = Image.fromarray(np_array, 'RGB')
    return img

def convert_to_RGB(cv_img):
    data = pil2numpy(cv_img)
    img = numpy2pil(data)
    return img


# if __name__ == '__main__':
#     data = pil2numpy()
#     img = numpy2pil(data)
#     img.show()