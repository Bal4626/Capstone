# Capstone

## The code to set up virtual env and run your files:

Installation For LINUX 

First, install uv if you don't have it: (type this into your terminal)

```
curl -LsSf https://astral.sh/uv/install.sh | sh
```
Then run this:
```
git clone https://github.com/wuphilipp/gello_software.git
cd gello_software
```
Then Run this 
```
uv venv --python 3.11
source .venv/bin/activate  
git submodule init
git submodule update
uv pip install -r requirements.txt
uv pip install -e .
uv pip install -e third_party/DynamixelSDK/python
```
If you want to exit just enter

```deactivate```

Each time you want to enter the virenv do:

```
cd gello_software
source .venv/bin/activate  
```

Now if you want to run files in the virenv, for example if you wanna run the script test.py located in /Downloads/test.py, you need to type:

```
python ~/Downloads/test.py
```

Control UR5 using gello/robots/ur.py through RTDE library

