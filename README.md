# AI_Search_algorithms 🧩

![Maze Screenshot](https://github.com/nimasayad37/AI_Search_algorithms/blob/main/mazepic.png?raw=true) 

## 📝 Description
This project implements classic AI search algorithms to solve mazes.  
It was developed as a course project for **Artificial Intelligence** at AmirKabir University of Technology.  

The project includes:  
- Maze generation (`mazeGenerator.py`) with only one best route.  
- Multiple search algorithms (`algorithms.py`): DFS, BFS, A*, Greedy, IDS.  
- Visualization of the algorithm solving the maze (`graphics.py`).  

---

## 🚀 Features
- Generate random mazes of any size.  
- Solve the maze using 5 AI search algorithms.  
- Visualize the solution path using a graphical interface.  

---

## 🛠️ Tech Stack
- Python 3.12   
- Libraries: 'matplotlib', 'stack_data'

---
files: 
  algorithms.py: this file contains the logic of the algorithms used, and the heuristic function that uses the euclidien distance.
  graphics.py: handles the graphics of the project and generates the results frame by frame
  mazeGenerator: creates the maze that the algorithms work on.
  main.py: the main file.
  AIProjectNo_2final.doc: an essay that goes into details on the projects and the results reached.

## 📦 Installation
```bash
git clone https://github.com/Nimasayad37/AI_Search_algorithms.git
pip install -r requirements.txt
run main.py!
