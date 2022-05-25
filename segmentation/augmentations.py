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
        self.operation = t.ColorJitter(brightness=.5, contrast=.3, saturation=.1)

    def __call__(self, image, target):
        image = self.operation(image)
        return image, target


class Normalize(object):
    def __init__(self):
        self.operation = t.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    
    def __call__(self, image, target):
        image = self.operation(image)
        return image, target

    
def collate_fn(batch):
    return tuple(zip(*batch))


inv_normalize = t.Normalize(mean=[-0.485/0.229, -0.456/0.224, -0.406/0.225], std=[1/0.229, 1/0.224, 1/0.225])