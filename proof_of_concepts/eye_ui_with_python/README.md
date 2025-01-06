# POC Eye visualisation in Python

## Installation steps required

Install python3, virtual environment and pygame
```bash
sudo apt update && sudo apt upgrade -y #update package manager
sudo apt install python3 python3-pip -y #intall python3 and pip3
sudo apt install python3-venv -y #install virtual environment for python
python3 -m venv venv # create a virtual envoirment named 'venv'
source venv/bin/activate # open the virtual environment

pip install --upgrade pip # (optional)
pip install pygame # install pygame
```

## Run the scripts
There are two POC created with python. "eyev1.py" and "eyev2.py".
Both script do the same, only "eyev2.py" shows the eye(s) in fullscreen. You can control the iris of a eye with keyboard arrows. Both scripts have the second eye commented out, because a second screen is needed for this. (Uncomment line 8, 62 and 66 in script "eyev2.py" or 8,47 and 51 in script "eyev1.py" when you have two HDMI screens available.)

To run the script
```bash
python3 ./eyev1.py 
#or
python3 ./eyev2.py
```
