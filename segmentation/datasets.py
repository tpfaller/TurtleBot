import os
import time
from augmentations import *
import numpy as np
from PIL import Image
from torch.utils.data import Dataset, DataLoader
import torchvision


class SegmentationDataset(Dataset):
	def __init__(self, root: str, transform=None, mode=None):
		[os.path.join(root, x) for x in os.listdir(root) if x.endswith('.jpg')]
		# self.images = sorted([os.path.join(root, x) for x in os.listdir(root) if x.endswith('.jpg')])
		# self.masks = sorted([os.path.join(root, x) for x in os.listdir(root) if x.endswith('truth.png')])
		self.masks = sorted([os.path.join(root, x) for x in os.listdir(root) if x.endswith('semantic_colored.png')])
		self.images = [x.replace('_label_ground-truth_semantic_colored.png', '.jpg') for x in self.masks]
		self.transform = transform
		
		if mode == 'turtlebot':
			self.classes = ['iron_man', 'captain_america', 'hulk', 'free_space', 'obstacles', 'wall']
			self.colors = [(0, 113, 188), (216, 82, 24), (236, 176, 31), (125, 46, 141), (118, 171, 47), (161, 19, 46)]
		elif mode == 'topdown':
			self.classes = ['iron_man', 'captain_america', 'hulk', 'free_space', 'obstacles', 'wall', 'turtlebot', 'background']
			self.colors = [(0, 113, 188), (216, 82, 24), (236, 176, 31), (125, 46, 141), (118, 171, 47), (161, 19, 46), (255, 0, 0), (0, 0, 0)]
        
		self.class_values = list(range(len(self.classes)))
		
	def __len__(self) -> int:
		return len(self.images)

	def __getitem__(self, index: int):
		image = Image.open(self.images[index])
		mask = np.array(Image.open(self.masks[index]))

		# masks = [(mask == v) for v in self.class_values]
		masks = [np.all(mask == c, axis=-1) for c in self.colors]
		# masks = [(mask == c) for c in self.colors]
		mask = np.stack(masks, axis=-1).astype('float')

		if self.transform:
			image, mask = self.transform(image, mask)
		return image, mask

	def get_num_classes(self) -> int:
		return len(self.classes)

	def get_colors(self) -> list:
		return self.colors


def main() -> None:
	pass


if __name__ == '__main__':
	main()
