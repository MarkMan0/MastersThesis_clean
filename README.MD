# Master's Thesis Cleaned version - Mark Pal
**Warning**
This is a stripped down version of the original project. The project history was deleted from this, along with proprietary files from Spinbotics. This project/repository contains only files written by me, or open-source libraries.
The project is still functional - the simulation still works, as before. Only files needed to communicate with the real hardware were removed.

## BotSimulation
To run the simulation, a virtual environment should be created. Before running the following commands, the file `logging.py` must be changed to a different name - the keyword `logging` interferes with pip or venv, and the virtual environment can't be created, while a file exists in the directory with this name. After creation and when the required packages are installed, the file can be changed back to it's original name.

```
python -m venv ./venv
pip install -r requirements.txt
```

The entry point is in the `main.py` file.

## Control
The project is compiled using Cmake. The main executable is called `SimpleControl`. Tested using MSVC compiler from Visual Studio 2022 and 2019. The project has dependencies on Googletest and Eigern - Cmake will download these dependencies.

To run the project in simulation mode, the simulation should be started first, and then SimpleControl.

To run on real hardware, define `USE_REAL_ROBOT` in `main.cpp`, line 17.
