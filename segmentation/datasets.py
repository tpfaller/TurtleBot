import os
import time

import cv2

from augmentations import *
import numpy as np
from PIL import Image
from torch.utils.data import Dataset, DataLoader
import torchvision


class SegmentationDataset(Dataset):
	def __init__(self, root: str, transform=None):
		[os.path.join(root, x) for x in os.listdir(root) if x.endswith('.jpg')]
		# self.images = sorted([os.path.join(root, x) for x in os.listdir(root) if x.endswith('.jpg')])
		# self.masks = sorted([os.path.join(root, x) for x in os.listdir(root) if x.endswith('truth.png')])
		self.masks = sorted([os.path.join(root, x) for x in os.listdir(root) if x.endswith('semantic_colored.png')])
		self.images = [x.replace('_label_ground-truth_semantic_colored.png', '.jpg') for x in self.masks]
		self.transform = transform
		self.classes = ['iron_man', 'captain_america', 'hulk', 'free_space', 'obstacles', 'wall']
		self.colors = [(0, 113, 188), (216, 82, 24), (236, 176, 31), (125, 46, 141), (118, 171, 47), (161, 19, 46)]
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
	aug = CustomCompose([ToTensor(), Resize(800,800), Jitter(), HorizontalFlip()])
	data = SegmentationDataset('data/v3', transform=aug)
	image_paths = data.images
	mask_paths = data.masks
	to_pil = torchvision.transforms.ToPILImage()
	loader = DataLoader(data, batch_size=1, shuffle=False)
	# images, targets = next(iter(loader))
	# for x, y in zip(images, targets):
	# 	img = torchvision.utils.draw_segmentation_masks(x.to(torch.uint8), masks=y.to(torch.bool), alpha=0.5)
	# 	to_pil(img).show()
	# 	to_pil(x).show()
	for index, (image, target) in enumerate(loader):
		cv2.namedWindow('Frame', cv2.WINDOW_KEEPRATIO)
		cv2.namedWindow('Mask', cv2.WINDOW_KEEPRATIO)
		print(f'Image {index}: \n',image_paths[index], '\n', mask_paths[index])
		cv2.imshow('Frame', image.squeeze().permute(1, 2, 0).numpy())
		mask= torchvision.utils.draw_segmentation_masks(image.squeeze().to(torch.uint8), masks=target.squeeze().to(torch.bool), alpha=0.5)
		cv2.imshow('Mask', mask.permute(1, 2, 0).numpy())
		cv2.resizeWindow('Frame', 600, 600)
		cv2.resizeWindow('Mask', 600, 600)
		time.sleep(0.8)
		if cv2.waitKey(25) & 0xFF == ord('q'):
			cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
