import torch

print([torch.cuda.device(i) for i in range(torch.cuda.device_count())])
