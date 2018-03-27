# Robot Motion Planning
## Plot and Navigate a Virtual Maze

### OVERVIEW

In this project, I programmed a micromouse(robot mouse) that finds an optimal way to reach a goal area in an unfamiliar virtual maze. The project is inspired by the world wide [micromouse competition](https://en.wikipedia.org/wiki/Micromouse) that holds almost every year.
The micromouse is allotted two(2) attempts with the maze; the initial attempt is for exploration, where the mouse keeps track of its position, discover walls, map out the maze and detect when it has reached the goal and the final attempt is for optimization, which is mainly for the mouse to find an optimal route and shortest possible time to get to the goal.

#### DEPENDENCIES
- [Python 2.7](https://www.python.org/downloads/)
- [NumPy](http://www.numpy.org/)

#### The starter code for this project was provided by Udacity and include the following files:
- **robot.py:** This script establishes the robot class and it contains the implemented code for this project.
- **maze.py:**  This script contains functions for constructing the maze and for checking for walls upon robot movement or sensing.
- **tester.py:** This script will be run to test the robot’s ability to navigate mazes.
- **showmaze.py:** This script can be used to create a visual demonstration of what a maze looks like.
- **test_maze_##.txt:** These consists of three(3) sample mazes to test the robot’s ability.

I created a file called **global_variables.py**, where I stored the global dictionaries (`dir_sensors`, `dir_move`, `dir_reverse`) and other variables I used for the implementation. I also added another maze (`test_maze_04.txt`) to test the model's performance on a different maze.

#### To display a visual representation of the maze
- In your terminal, run `python showmaze.py test_maze_##.txt`

#### To run the robot through a maze
- In your terminal, run `python tester.py test_maze_##.txt`
