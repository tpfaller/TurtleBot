import random
import torch
import torchvision.transforms.functional as tf
from torchvision import transforms as t


class CustomCompose(object):
    def __init__(self, transforms):
        self.operations = transforms
        
    def __call__(self, image, target):
        for op in self.operations:
            image, target = op(image, target)
        return image, target


class Resize(object):
    def __init__(self, width: int, height: int):
        self.resize_image = t.Resize((width, height))
        self.resize_mask = t.Resize((width, height), interpolation=tf.InterpolationMode('nearest'))  # Image.NEAREST)
        
    def __call__(self, image: torch.Tensor, mask: torch.Tensor):
        return self.resize_image(image), self.resize_mask(mask)


class RandomCrop(object):
    def __init__(self, minimum_size: float = .3):
        self.min = minimum_size

    def __call__(self, image: torch.Tensor, mask: torch.Tensor):
        c, h, w = image.size()
        height, width = random.randint(int(h*self.min), h), random.randint(int(w*self.min), w)
        top, left = random.randint(0, h-height), random.randint(0, w-width)
        image = tf.crop(image, top, left, height, width)
        mask = tf.crop(mask, top, left, height, width)
        return image, mask


class ToTensor(object):
    def __init__(self):
        self.operation = t.ToTensor()
        
    def __call__(self, image, target):
        image = self.operation(image)
        target = self.operation(target)
        return image, target


class HorizontalFlip(object):
    def __init__(self):
        self.threshold = .5

    def __call__(self, image, target):
        if random.random() > self.threshold:
            image = tf.hflip(image)
            target = tf.hflip(target)
        return image, target


class Jitter(object):
    def __init__(self):
        self.operation = t.ColorJitter(brightness=.7, contrast=.4, hue=.1, saturation=.2)

    def __call__(self, image, target):
        image = self.operation(image)
        return image, target


class Normalize(object):
    def __init__(self):
        self.operation = t.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    
    def __call__(self, image, target):
        image = self.operation(image)
        return image, target


class PreProcess(object):
    def __init__(self):
        self.width, self.height = 400, 400
        self.operations = t.Compose([t.ToTensor(), t.Resize((self.width, self.height)),
                                     t.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])

    def __call__(self, image):
        image = self.operations(image)
        return image.unsqueeze(0)


class InvertNormalization(object):
    def __init__(self):
        self.invert = t.Normalize(mean=[-0.485 / 0.229, -0.456 / 0.224, -0.406 / 0.225],
                                  std=[1 / 0.229, 1 / 0.224, 1 / 0.225])

    def __call__(self, image):
        return self.invert(image)

    
def collate_fn(batch):
    return tuple(zip(*batch))


if __name__ == '__main__':
    x = torch.rand(3, 640, 480)
    y = torch.rand(3, 640, 480)
    crop = RandomCrop()
    print(crop(x, y)[0].size())
