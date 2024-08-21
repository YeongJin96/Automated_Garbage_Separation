import torch
print('torch_version: ', torch.__version__)


print(torch.cuda.is_available())

print(torch.cuda.current_device())

print(torch.cuda.get_device_name(0))