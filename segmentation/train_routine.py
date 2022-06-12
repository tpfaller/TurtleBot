import argparse
import math
import os
import sys
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from augmentations import *
from datasets import SegmentationDataset
from models import get_model
from utils import create_dir, bool_flag
from torchvision.utils import draw_segmentation_masks, make_grid


def create_images(images, targets, predictions, writer, epoch) -> None:
    global COLORS
    inv = InvertNormalization()
    images = [inv(x) for x in images]
    gt = [draw_segmentation_masks(x.to(torch.uint8), y.to(torch.bool), alpha=0.5, colors=COLORS) for x, y in zip(images, targets)]
    preds = [draw_segmentation_masks(x.to(torch.uint8), y.to(torch.bool), alpha=0.5, colors=COLORS) for x, y in zip(images, predictions)]
    grid = make_grid(images+gt+preds, nrow=len(images))
    writer.add_image(f'Epoch_{epoch}', grid)
    writer.flush()


def IoU(preds, targets) -> float:
    N, c, _, _ = preds.size()
    intersection = torch.logical_and(preds, targets).view(N,-1).sum(dim=-1).squeeze_()
    union = torch.logical_or(preds, targets).view(N,-1).sum(dim=-1).squeeze_()
    IoU = torch.divide(intersection, union)
    return torch.mean(IoU).item()


def eval_one_epoch(model, criterion, loader, device, writer: SummaryWriter, epoch) -> dict:
    model.eval()
    show_pred_flag, epoch_iou, epoch_loss = True, .0, .0
    for ix, (images, targets) in enumerate(loader):
        images, targets = images.to(device), targets.to(device)
        output = model(images)['out']
        epoch_loss += criterion(output, targets).item()
        output = output.detach().cpu()
        idx = torch.argmax(output, dim=1, keepdim=True)
        preds = torch.zeros_like(output).scatter_(1, idx, 1.)
        epoch_iou += IoU(preds, targets.detach().cpu())
        if show_pred_flag:
            create_images(images.detach().to('cpu'), targets.detach().to('cpu'), preds.detach().to('cpu'), writer, epoch)
            show_pred_flag = False
    epoch_loss /= len(loader)
    epoch_iou /= len(loader)
    print("\n----------\nValid\n----------")
    print('IoU: ', epoch_iou, '\nEpoch Loss: ', epoch_loss)
    writer.add_scalar('Val/IoU', epoch_iou, epoch)
    writer.add_scalar('Val/Loss', epoch_loss, epoch)
    return {'Epoch_Loss': epoch_loss, 'Epoch_IoU': epoch_iou}


def train_one_epoch(model, criterion, optimizer, scheduler, loader, fp16_scaler, device, writer, epoch) -> None:
    model.train()
    epoch_loss, epoch_iou = .0, .0
    for ix, (images, targets) in enumerate(loader):
        images = images.to(device)
        targets = targets.to(device)

        with torch.cuda.amp.autocast(fp16_scaler is not None):
            # optimizer.zero_grad()
            output = model(images)['out']
            loss = criterion(output, targets)

        if not math.isfinite(loss.item()):
            print("Loss is {}, stopping training".format(loss.item()))
            sys.exit(1)

        # student update
        optimizer.zero_grad()
        param_norms = None
        if fp16_scaler is None:
            loss.backward()
            optimizer.step()
        else:
            fp16_scaler.scale(loss).backward()
            fp16_scaler.step(optimizer)
            fp16_scaler.update()

        epoch_loss += loss.item()
        output = output.detach().cpu()
        idx = torch.argmax(output, dim=1, keepdim=True)
        preds = torch.zeros_like(output).scatter_(1, idx, 1.)
        epoch_iou += IoU(preds, targets.detach().cpu())
    epoch_loss /= len(loader)
    epoch_iou /= len(loader)
    print("\n----------\nTrain\n----------")
    print('IoU: ', epoch_iou, '\nEpoch Loss: ', epoch_loss)
    writer.add_scalar('Train/IoU', epoch_iou, epoch)
    writer.add_scalar('Train/Loss', epoch_loss, epoch)
    scheduler.step()


def train_model(args) -> None:
    ckp_dir = create_dir(args.ckp_dir, f"{args.arch}-{args.mode}")
    writer = SummaryWriter(log_dir=os.path.join(ckp_dir, 'runs'))
    width, height = 400, 400
    transform_train = CustomCompose([ToTensor(), RandomCrop(), Resize(width, height), Jitter(),
                                     HorizontalFlip(), Normalize()])
    transform_val = CustomCompose([ToTensor(), Resize(width, height), Normalize()])

    data_train = SegmentationDataset(args.data_dir, transform_train, args.mode)
    n_classes = data_train.get_num_classes()
    n_images = len(data_train)
    data_val = SegmentationDataset(args.data_dir, transform_val, args.mode)
    indices = torch.randperm(len(data_train)).tolist()
    data_train = torch.utils.data.Subset(data_train, indices[:int(n_images*.8)])
    data_val = torch.utils.data.Subset(data_val, indices[int(n_images*.8):])

    loader_train = DataLoader(data_train, batch_size=args.batch_size, shuffle=True, drop_last=True, pin_memory=False)
    loader_val = DataLoader(data_val, batch_size=args.batch_size, shuffle=False, drop_last=True, pin_memory=False)

    model = get_model(args, n_classes)

    if args.weights_dir != 'none':
        model.load_state_dict(torch.load(args.weights_dir))

    device = 'cuda:0' if torch.cuda.is_available() else 'cpu'

    if args.mode == 'turtlebot':
        weight = torch.cat([torch.ones((1, 3, width, height)).mul_(20.0), torch.ones((1, 3, width, height))], dim=1).to(device)
    elif args.mode == 'topdown':
        weight = torch.cat([torch.ones((1, 3, width, height)).mul_(10.0), torch.ones((1, 3, width, height)), torch.ones((1, 1, width, height)).mul_(5.0), torch.ones((1, 1, width, height))], dim=1).to(device)

    criterion = torch.nn.BCEWithLogitsLoss(weight=weight)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.learning_rate, betas=(args.momentum, args.alpha),
                                  weight_decay=args.weight_decay)
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=args.step_size, gamma=args.gamma)
    fp16_scaler = torch.cuda.amp.GradScaler() if args.use_fp16 and torch.cuda.is_available() else None

    model.to(device)

    metrics = dict()
    for epoch in range(1, args.n_epochs +1):
        print(f'\n------------------\nEpoch[{epoch:>3d}/{args.n_epochs:>3d}]:')
        train_one_epoch(model, criterion, optimizer, scheduler, loader_train, fp16_scaler, device, writer, epoch)
        if epoch % args.eval_epoch == 0 or epoch == args.n_epochs:
            metrics = eval_one_epoch(model, criterion, loader_val, device, writer, epoch)
            torch.save(model.state_dict(), os.path.join(ckp_dir, f'{args.arch}_{epoch}.pth'))

    writer.add_hparams(hparam_dict=args.__dict__, metric_dict=metrics)


def main() -> None:
    global COLORS

    parser = argparse.ArgumentParser()
    parser.add_argument('--data_dir', type=str, default='data/v4')
    parser.add_argument('--ckp_dir', type=str, default='checkpoints')
    parser.add_argument('--arch', type=str, default='deeplab',
                        choices=['deeplab', 'lraspp'])
    parser.add_argument('--mode', type=str, default='turtlebot',
                        choices=['turtlebot', 'topdown'])
    parser.add_argument('--weights_dir', type=str, default='none')

    # Trainingsparameter
    parser.add_argument('--learning_rate', type=float, default=1e-2)
    parser.add_argument('--momentum', type=float, default=.9)
    parser.add_argument('--alpha', type=float, default=.99)
    parser.add_argument('--weight_decay', type=float, default=.0)
    parser.add_argument('--step_size', type=int, default=1)
    parser.add_argument('--gamma', type=float, default=0.9)
    parser.add_argument('--batch_size', type=int, default=2)
    parser.add_argument('--n_epochs', type=int, default=10)

    parser.add_argument('--use_fp16', type=bool_flag, default=False)
    parser.add_argument('--eval_epoch', type=int, default=5)
    args = parser.parse_args()

    if args.mode == "turtlebot":
        COLORS = [(0, 113, 188), (216, 82, 24), (236, 176, 31), (125, 46, 141), (118, 171, 47), (161, 19, 46)]
    elif args.mode == "topdown":
        COLORS = [(0, 113, 188), (216, 82, 24), (236, 176, 31), (125, 46, 141), (118, 171, 47), (161, 19, 46), (255, 0, 0), (0, 0, 0)]
    train_model(args)


if __name__ == '__main__':
    main()
