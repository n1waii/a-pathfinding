/*
    A demonstration of the A* Search Algorithm.
    All work is written by the author except C++ STD library functions.
*/

#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <map>
#include <limits>
#include <queue>
#include <iterator>
#include <stdlib.h>
#include <ctime>
#include <unistd.h>
#include <termios.h>
#include <algorithm>

const int RECONSTRUCTION_DELAY_MS = 400; // change this to 0 to see how fast it constructs the path.  
const int COLLUMNS = 80; const int ROWS = 30;
int Y_START = 15; int X_START = 40;
int Y_AGENT = Y_START; int X_AGENT = X_START-1;
int Y_GOAL; int X_GOAL;

int grid[ROWS][COLLUMNS];

typedef std::pair<int, int> nodeTemplate;

struct nodeComparison { 
    constexpr bool operator()( 
        std::pair<nodeTemplate, int> &a, 
        std::pair<nodeTemplate, int> &b)
        const noexcept
    { 
        return a.second > b.second; 
    }
}; 

void initiate();
void render();
int heuristic(nodeTemplate goal, nodeTemplate node);
std::vector<nodeTemplate> search();
std::vector<nodeTemplate> tracePath(std::map<nodeTemplate, nodeTemplate> cameFrom, nodeTemplate current);
std::vector<nodeTemplate> getNeighbors(nodeTemplate node);

int steps = 0;

int main() {
    struct termios old_tio, new_tio;

    // Detect keypresses without having to press enter 
    tcgetattr(STDIN_FILENO,&old_tio);
    new_tio=old_tio;
    new_tio.c_lflag &=(~ICANON & ~ECHO);
    tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);

    initiate();
    render();

    char c;
    while (c != 'p') {
        c = getchar();
        short int lastState = grid[Y_AGENT][X_AGENT];
        switch (c) {
            case 'w':
                Y_AGENT = std::clamp(Y_AGENT-1, 0, ROWS);
                break;
            case 'a':
                X_AGENT = std::clamp(X_AGENT-1, 0, COLLUMNS);
                break;
            case 's':
                Y_AGENT = std::clamp(Y_AGENT+1, 0, ROWS);
                break;
            case 'd':
                X_AGENT = std::clamp(X_AGENT+1, 0, COLLUMNS);
                break;
        }
        short int newState = grid[Y_AGENT][X_AGENT];
        if (newState != 2 || newState != 7) {
            grid[Y_AGENT][X_AGENT] = 3;
            render();
        } else {
          grid[Y_AGENT][X_AGENT] = lastState;
        }
    }
    
    std::vector<nodeTemplate> path = search();
    nodeTemplate lastNode(Y_START, X_START);
    short int dir = 1;
    for (std::vector<nodeTemplate>::iterator node = path.begin(); node != path.end(); ++node) {
        // get direction delta for displaying array in correct orientation
        if (node->first > lastNode.first) {
            dir = 6; // up movement
        } else if (node->first < lastNode.first) {
            dir = 5; // down movement
        } else if (node->second > lastNode.second) {
            dir = 4; // right movement
        } else if (node->second < lastNode.second) {
            dir = 1; // left movement
        }
        lastNode = *node;
        if (grid[node->first][node->second] != 7) {
            grid[node->first][node->second] = dir;
        }
        render();
        steps++;
        std::this_thread::sleep_for(std::chrono::milliseconds(RECONSTRUCTION_DELAY_MS)); // wait to see each step
    }
}

void initiate() {
    std::srand(std::time(nullptr));
    Y_GOAL = std::rand() % ROWS;
    X_GOAL = std::rand() % COLLUMNS;
    steps = 0;

    for (int y = 0; y < ROWS; y++) {
        for (int x = 0; x < COLLUMNS; x++) {
            grid[y][x] = 0;
        }
    }

    grid[Y_START][X_START] = 7;
    grid[Y_GOAL][X_GOAL] = 2;
}

int heuristic(nodeTemplate goal, nodeTemplate node) {
    const int D = 1;
    int dx = abs(node.second - goal.second);
    int dy = abs(node.first - goal.first);
    return D * (dx+dy);
}

std::vector<nodeTemplate> tracePath(std::map<nodeTemplate, nodeTemplate> cameFrom, nodeTemplate current) {
    std::vector<nodeTemplate> path;
    while (cameFrom.find(current) != cameFrom.end()) {
        current = cameFrom.find(current)->second;
        path.insert(path.begin(), current);
    }
    return path;
}

std::vector<nodeTemplate> getNeighbors(nodeTemplate node) {
    std::vector<nodeTemplate> neighbors;
    
    if (node.second != 0 && (grid[node.first][node.second-1] == 0 || grid[node.first][node.second-1] == 2)) { // West
        neighbors.push_back(std::make_pair(node.first, node.second-1));
    }
    if (node.second != COLLUMNS && (grid[node.first][node.second+1] == 0 || grid[node.first][node.second+1] == 2)) { // East
        neighbors.push_back(std::make_pair(node.first, node.second+1));
    }
    if (node.first != 0 && (grid[node.first-1][node.second] == 0 || grid[node.first-1][node.second] == 2)) { // North
        neighbors.push_back(std::make_pair(node.first-1, node.second));
    }
    if (node.first != ROWS && (grid[node.first+1][node.second] == 0 || grid[node.first+1][node.second] == 2)) { // South
        neighbors.push_back(std::make_pair(node.first+1, node.second));
    }

    return neighbors;
}

void render() {
    std::cout << "\033[2J\033[0;0H"; // clear terminal
    for (int y = 0; y < ROWS; y++) {
        for (int x = 0; x < COLLUMNS; x++) {
            int state = grid[y][x];
            switch (state) {
                case 0:
                    std::cout << "*"; // ground
                    break;
                case 1:
                    std::cout << "⮜"; // left arrow
                    break;
                case 2:
                    std::cout << "★"; // goal
                    break;
                case 3:
                    std::cout << "⬜"; // wall
                    break;
                case 4:
                    std::cout << "⮞"; // right arrow
                    break;
                case 5:
                    std::cout << "⮝"; // up arrow
                    break;
                case 6:
                    std::cout << "⮟"; // down arrow
                    break;
                case 7:
                    std::cout << "◉";
            }
        }
        std::cout << "\n";
    }
    std::cout << "Steps: " << steps << "\n";
} 

std::vector<nodeTemplate> search() {
    nodeTemplate start = std::make_pair(Y_START, X_START);
    nodeTemplate goal = std::make_pair(Y_GOAL, X_GOAL);

    std::priority_queue<
        std::pair<nodeTemplate, int>, 
        std::vector<std::pair<nodeTemplate, int>>,
        nodeComparison
    > frontier;
    
    frontier.push(std::make_pair(start, 0));

    std::map<nodeTemplate, bool> explored;
    std::map<nodeTemplate, nodeTemplate> cameFrom;
    std::map<nodeTemplate, int> gScore {
        {start, 0}
    };
    std::map<nodeTemplate, int> fScore {
        {start, heuristic(goal, start)}
    };

    while (!frontier.empty()) {
        nodeTemplate n = frontier.top().first;
        frontier.pop(); 
        if (n.first == goal.first && n.second == goal.second) { // found goal
            std::cout << "Found goal\n";
            return tracePath(cameFrom, n);
        }

        std::vector<nodeTemplate> neighbors = getNeighbors(n);
        for (std::vector<nodeTemplate>::iterator neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor) {
            if (explored.find(*neighbor) != explored.end()) {
                continue;
            }
            explored.emplace(*neighbor, true);
            int tentative_gScore = (gScore.find(n)->second) + 1;
            int neighbor_gScore = (gScore.find(*neighbor) != gScore.end() ? gScore.find(*neighbor)->second : std::numeric_limits<int>::max());
            if (tentative_gScore < neighbor_gScore) {
                int c = heuristic(goal, *neighbor); 

                cameFrom.emplace(*neighbor, n);
                gScore.emplace(*neighbor, tentative_gScore);
                fScore.emplace(*neighbor, tentative_gScore + c);

                frontier.push(std::make_pair(*neighbor, tentative_gScore+c));
            }
        }
    }
    std::cout << "No goal found\n";
    return std::vector<nodeTemplate>();
}
