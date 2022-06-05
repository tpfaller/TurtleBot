import argparse
import torch
from torch import nn
import torchvision.models.segmentation as seg_models


def get_model(args, n_classes: int):
    if args.arch == 'deeplab':
        model = seg_models.deeplabv3_mobilenet_v3_large(pretrained=True)
        model.classifier[4] = nn.Conv2d(256, n_classes, 1, stride=1)
        model.aux_classifier[4] = nn.Conv2d(10, n_classes, 1, stride=1)
    elif args.arch == 'lraspp':
        model = seg_models.lraspp_mobilenet_v3_large()
        model.classifier.low_classifier = nn.Conv2d(40, n_classes, 1, stride=1)
        model.classifier.high_classifier = nn.Conv2d(128, n_classes, 1, stride=1)
    return model


def load_pretrained_model(args):
    model = get_model(args, 6)
    state_dict_ = torch.load(args.weights_dir)
    model.load_state_dict(state_dict_)
    return model


def inference(model, image: torch.Tensor) -> torch.Tensor:
    return model(image.unsqueeze_(0))['out'].squeeze_()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights_dir', type=str, default='checkpoints/lraspp_25_05_2022-21_58/lraspp.pth')
    parser.add_argument('--arch', type=str, default='lraspp',
                        choices=['deeplab', 'lraspp'])
    args = parser.parse_args()
    model = load_pretrained_model(args)
    img = torch.rand((3, 400, 400))
    y = inference(model, img)
    print(y.size())


if __name__ == '__main__':
    main()
