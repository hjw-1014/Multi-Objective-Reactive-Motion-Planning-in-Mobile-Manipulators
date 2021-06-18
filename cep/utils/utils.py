import numpy as np
import torch


def numpy2torch(x, device='cpu'):
    return torch.from_numpy(x).float().to(device)

def torch2numpy(x):
    if x is None:
        print(x)

    if x.device.type=='cuda':
        return x.cpu().detach().numpy()
    else:
        return x.detach().numpy()
