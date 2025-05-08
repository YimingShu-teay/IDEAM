# IDEAM

This repository is the implementation of the TITS'25 paper:

**Agile Decision-Making and Safety-Critical Motion Planning for Emergency Autonomous Vehicles**

### Installation

---

#### Create conda env

```shell
conda create -n IDEAM python=3.7
```

#### Installation dependency

The `requirements.txt` file should list all Python libraries and they will be installed using:

```bash
pip install -r requirements.txt
```

### Usage

---

#### **One random scenario test** 

```shell
python solid_test.py
```

#### **Parallel random scenarios test** 

```shell
python multi_test.py
```

#### Emergency scenarios test

```shell
python emergency_test.py
```

### Citation

---

