# IDEAM

This repository is the implementation of the TITS'25 paper:

**Agile Decision-Making and Safety-Critical Motion Planning for Emergency Autonomous Vehicles**

If you feel interested, ▶️ [Click to watch the video](https://www.youtube.com/watch?v=873BZoQSf-Q)

<img src="assets/demo.gif" alt="demo" width="300" height="auto" />

## Installation

### Create conda env

```shell
conda create -n IDEAM python==3.7
```

### Install dependency

The `requirements.txt` file should list all Python libraries and they will be installed using:

```bash
pip install -r requirements.txt
```



## Usage

### **One random scenario test** 

```shell
python solid_test.py
```

### **Parallel random scenarios test** 

```shell
python multi_test.py
```

### Emergency scenarios test

```shell
python emergency_test.py
```

### Notes:

【1】You can use the 200 pre-randomly generated configuration files in `file_save` for testing, or alternatively, randomly generate new scenarios for testing.

【2】When recording the time consumption, you need to separately track the statistics for C-DFS and LSGM in the same scene, as LSGM includes C-DFS, using `openpyxl` within the LSGM will result in incorrect duration statistics. Alternatively, you can use a different statistical method.

【3】Here are some ppts in `records`,  check them if you find them useful.

## Citation

If you find our code and paper can help, please cite our paper as:

To be finished...

## Acknowledgment

IDEAM is greatly inspired by the following contributions to the open-source community: [iterative-MPC-DHOCBF](https://github.com/ShockLeo/Iterative-MPC-DHOCBF.git), [Safety-Critical System](https://github.com/YimingShu-teay/Safety-critical-Decision-making-and-Control.git), [MPC-CBF](https://github.com/HybridRobotics/MPC-CBF.git), [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics.git)

