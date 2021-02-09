/*
    A demonstration of the A* Search Algorithm. 
*/

#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <map>
#include <limits>
#include <queue>
#include <iterator>

const int COLLUMNS = 80, ROWS = 30;
const int Y_START = 5, X_START = 7;
const int Y_GOAL = 20, X_GOAL = 50;

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

int main() {
    initiate();
    grid[Y_START][X_START] = 1;
    grid[Y_GOAL][X_GOAL] = 2;

    std::vector<nodeTemplate> path = search();
    for (std::vector<nodeTemplate>::iterator node = path.begin(); node != path.end(); ++node) {
        grid[node->first][node->second] = 1;
        render();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void initiate() {
    for (int y = 0; y < ROWS; y++) {
        for (int x = 0; x < COLLUMNS; x++) {
            grid[y][x] = 0;
        }
    }
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
            if (state == 0) { // Land
                std::cout << "*";
            } else if (state == 1) { // Agent
                std::cout << "#";
            } else if (state == 2) { // Goal
                std::cout << "★";
            }
        }
        std::cout << "\n";
    }
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
       // std::this_thread::sleep_for(std::chrono::milliseconds(50));
        nodeTemplate n = frontier.top().first;
        frontier.pop(); 
        //grid[n.first][n.second] = 1;
        //render();
        std::cout << n.first << " | " << n.second << "\n";
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
