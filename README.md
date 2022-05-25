# Turtlebot

## setup

external dependencies 

```
sudo apt-get install python3-dev
sudo apt-get install python3-venv
python3 -m venv venv
```

_CUDA 11.0_
```
source venv/bin/activate
pip install --upgrade pip
pip install torch==1.9.0+cu111 torchvision==0.10.0+cu111 torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html
pip install -r requirements.txt
```

_CUDA 10.1_
```
source venv/bin/activate
pip install --upgrade pip
pip install torch==1.9.0+cu101 torchvision==0.10.0+cu101 torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html
pip install -r requirements.txt
```

_CPU_
```
source venv/bin/activate
pip install --upgrade pip
pip install torch==1.9.0+cpu torchvision==0.10.0+cpu torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html
pip install -r requirements.txt
```