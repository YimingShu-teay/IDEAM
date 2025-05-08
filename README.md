# IDEAM

This repository is the implementation of the TITS'25 paper:

**Agile Decision-Making and Safety-Critical Motion Planning for Emergency Autonomous Vehicles**

If you feel interested, ▶️ [Click to watch the video](https://www.youtube.com/watch?v=873BZoQSf-Q)

<img src="assets\demo.gif" alt="demo" style="zoom:50%;" />

## Installation

### Create conda env

```shell
conda create -n IDEAM python=3.7
```

### Installation dependency

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

### Note

【1】You can use the 200 pre-randomly generated configuration files in `file_save` for testing, or alternatively, randomly generate new scenarios for testing.

【2】When recording the time consumption, you need to separately track the statistics for C-DFS and LSGM in the same scene, as LSGM includes C-DFS, using `openpyxl.load_workbook` within the LSGM will result in incorrect duration statistics. Alternatively, you can use a different statistical method.

## Citation

To be finished...

