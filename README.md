# 3x3 Grid Search Algorithms Implementation

This repository contains a solution to a 3x3 grid-based sliding puzzle problem. The solution includes the implementation of various search algorithms like A*, BFS, IDA*, DFID, and DFBnB. The project is designed to identify the optimal sequence of moves from a given initial state to a goal state while adhering to specific constraints.

## Problem Description
The problem involves moving marbles of three colors (red, green, and blue) on a circular 3x3 grid:
- The grid contains 6 marbles and empty spaces.
- Marbles can be moved horizontally or vertically but not diagonally.
- The goal is to transform the grid from an initial state to a target state while minimizing the total movement cost.

### Rules and Constraints
1. Movement cost:
   - Red: 3 units
   - Green: 1 unit
   - Blue: 1 unit
2. Moves are allowed only to empty cells and cannot move into black (blocked) cells.
3. The grid wraps around circularly (e.g., a marble moving right off the edge reappears on the left).

## Features
- **Multiple Search Algorithms**: 
  - Breadth-First Search (BFS)
  - Depth-First Iterative Deepening (DFID)
  - Iterative Deepening A* (IDA*)
  - Depth-First Branch and Bound (DFBnB)
  - A* Search
- **Heuristic Functions**:
  - Designed and evaluated for admissibility and consistency.
- **Performance Metrics**:
  - Number of nodes generated.
  - Solution cost.
  - Execution time.

## Input and Output
### Input (`input.txt`)
1. First line: Algorithm name (`BFS`, `DFID`, `IDA*`, `A*`, or `DFBnB`).
2. Second line: Time output option (`with time` or `no time`).
3. Third line: Open list output option (`with open` or `no open`).
4. Next lines: Initial state of the grid.
5. Last line: Goal state of the grid.

Example:

### input (`input.txt`)
BFS
with time
no open
R,R,_
B,B,_
G,G,X
Goal state:
G,R,R
B,B,_
_,G,X

### Output (`output.txt`)
(1,1):R:(1,3)--(3,1):G:(1,1)
Num: 11
Cost: 13
0.002 seconds

