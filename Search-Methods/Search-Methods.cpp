#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <stack>
#include <chrono>
#include <cmath>
#include <functional>
#include <unordered_set>

using namespace std;

struct City {
     string name;
     double latitude;
     double longitude;
};

class RouteFinder {
public:
     // Function to load cities and their adjacencies from files
     void loadCities(const string& citiesFile);
     void loadAdjacencies(const string& adjacenciesFile);

     // Function to find a route using breadth-first search
     void breadthFirstSearch(const string& start, const string& end);

     // Function to find a route using depth-first search
     void depthFirstSearch(const string& start, const string& end);

     // Function to find a route using iterative deepening depth-first search
     void iterativeDeepeningDFS(const string& start, const string& end);

     // Function to find a route using best-first search
     void bestFirstSearch(const string& start, const string& end);

     // Function to find a route using A* search
     void aStarSearch(const string& start, const string& end);

     // Helper function to calculate distance between two cities
     double calculateDistance(const City& city1, const City& city2);
private:
     unordered_map<string, City> cityMap;
     unordered_map<string, vector<string>> adjacencyMap;

     void depthFirstSearchHelper(const string& currentCity, const string& end,
          unordered_map<string, bool>& visited,
          unordered_map<string, string>& parent,
          unordered_map<string, double>& distanceMap,
          bool& routeFound);

     void iterativeDeepeningDFSHelper(const string& currentCity, const string& end,
          unordered_map<string, bool>& visited,
          unordered_map<string, string>& parent,
          unordered_map<string, double>& distanceMap,
          int depthLimit, bool& routeFound);

     // Helper function to print the route
     void printRoute(const vector<string>& route);

     // Helper function to print route details
     void printRouteDetails(const vector<string>& route, double distance, double time);

     // Helper function to get current timestamp
     long long getCurrentTime();
};

// Implement the functions here... 
// Helper function to get current timestamp in microseconds
long long RouteFinder::getCurrentTime() {
     auto currentTime = chrono::high_resolution_clock::now().time_since_epoch();
     return chrono::duration_cast<chrono::microseconds>(currentTime).count();
}

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper function to calculate distance between two cities using Haversine formula
double RouteFinder::calculateDistance(const City& city1, const City& city2) {
     // Radius of the Earth in kilometers
     const double R = 6371.0;

     // Convert latitude and longitude from degrees to radians
     double lat1Rad = city1.latitude * M_PI / 180.0;
     double lon1Rad = city1.longitude * M_PI / 180.0;
     double lat2Rad = city2.latitude * M_PI / 180.0;
     double lon2Rad = city2.longitude * M_PI / 180.0;

     // Haversine formula
     double dlat = lat2Rad - lat1Rad;
     double dlon = lon2Rad - lon1Rad;
     double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1Rad) * cos(lat2Rad) * sin(dlon / 2) * sin(dlon / 2);
     double c = 2 * atan2(sqrt(a), sqrt(1 - a));
     double distance = R * c;

     return distance;
}

// Function to find a route using A* search
void RouteFinder::aStarSearch(const string& start, const string& end) {
     auto startTime = chrono::high_resolution_clock::now();

     priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> pq;
     unordered_map<string, double> costSoFar;
     unordered_map<string, string> parent;

     pq.push({ calculateDistance(cityMap[start], cityMap[end]), start });
     costSoFar[start] = 0.0;

     bool routeFound = false;

     while (!pq.empty()) {
          double currentCost = pq.top().first;
          string currentCity = pq.top().second;
          pq.pop();

          if (currentCity == end) {
               routeFound = true;
               break;
          }

          for (const string& neighbor : adjacencyMap[currentCity]) {
               double newCost = costSoFar[currentCity] + calculateDistance(cityMap[currentCity], cityMap[neighbor]);

               if (costSoFar.find(neighbor) == costSoFar.end() || newCost < costSoFar[neighbor]) {
                    costSoFar[neighbor] = newCost;
                    double priority = newCost + calculateDistance(cityMap[neighbor], cityMap[end]);
                    pq.push({ priority, neighbor });
                    parent[neighbor] = currentCity;
               }
          }
     }

     auto endTime = chrono::high_resolution_clock::now();
     auto duration = chrono::duration_cast<chrono::microseconds>(endTime - startTime).count();

     if (routeFound) {
          vector<string> route;
          string currentCity = end;

          while (!currentCity.empty()) {
               route.push_back(currentCity);
               currentCity = parent[currentCity];
          }

          reverse(route.begin(), route.end());

          printRoute(route);
          double distance = costSoFar[end]; // Total cost is the distance for A* search
          printRouteDetails(route, distance, duration / 1e6); // Convert microseconds to seconds
     }
     else {
          cout << "No route found from " << start << " to " << end << "." << endl;
     }
}

// Function to find a route using best-first search
void RouteFinder::bestFirstSearch(const string& start, const string& end) {
     auto startTime = chrono::high_resolution_clock::now();

     struct Compare {
          RouteFinder& routeFinder;
          unordered_map<string, City>& cityMap;
          string end;

          explicit Compare(RouteFinder& routeFinder, const string& end)
               : routeFinder(routeFinder), cityMap(routeFinder.cityMap), end(end) {}

          bool operator()(const string& city1, const string& city2) {
               double dist1 = routeFinder.calculateDistance(cityMap[city1], cityMap[end]);
               double dist2 = routeFinder.calculateDistance(cityMap[city2], cityMap[end]);

               return dist1 > dist2;  // Min-heap, so we reverse the comparison
          }
     };

     unordered_map<string, bool> visited;
     unordered_map<string, string> parent;
     unordered_map<string, double> distanceMap;  // New map to store distances

     Compare compare(*this, end);  // Create an instance of Compare with a reference to the RouteFinder
     priority_queue<string, vector<string>, Compare> pq(compare);

     pq.push(start);
     visited[start] = true;
     distanceMap[start] = 0.0;  // Starting distance is 0

     bool routeFound = false;

     while (!pq.empty()) {
          string currentCity = pq.top();
          pq.pop();

          if (currentCity == end) {
               routeFound = true;
               break;
          }

          for (const string& neighbor : adjacencyMap[currentCity]) {
               if (!visited[neighbor]) {
                    pq.push(neighbor);
                    visited[neighbor] = true;
                    parent[neighbor] = currentCity;

                    // Accumulate distance
                    distanceMap[neighbor] = distanceMap[currentCity] + calculateDistance(cityMap[currentCity], cityMap[neighbor]);
               }
          }
     }

     auto endTime = chrono::high_resolution_clock::now();
     auto duration = chrono::duration_cast<chrono::microseconds>(endTime - startTime).count();

     if (routeFound) {
          vector<string> route;
          string currentCity = end;

          while (!currentCity.empty()) {
               route.push_back(currentCity);
               currentCity = parent[currentCity];
          }

          reverse(route.begin(), route.end());

          printRoute(route);
          double distance = distanceMap[end];  // Total distance is the last recorded distance
          printRouteDetails(route, distance, duration / 1e6);  // Convert microseconds to seconds
     }
     else {
          cout << "No route found from " << start << " to " << end << "." << endl;
     }
}


// Function to find a route using iterative deepening depth-first search
void RouteFinder::iterativeDeepeningDFS(const string& start, const string& end) {
     auto startTime = chrono::high_resolution_clock::now();

     bool routeFound = false;
     int depthLimit = 0;
     unordered_map<string, string> parent;
     unordered_map<string, double> distanceMap;  // New map to store distances

     while (!routeFound) {
          unordered_map<string, bool> visited;

          iterativeDeepeningDFSHelper(start, end, visited, parent, distanceMap, depthLimit, routeFound);

          depthLimit++;

          if (depthLimit > cityMap.size()) {
               break;  // To avoid infinite loop in case of no route
          }
     }

     auto endTime = chrono::high_resolution_clock::now();
     auto duration = chrono::duration_cast<chrono::microseconds>(endTime - startTime).count();

     if (routeFound) {
          vector<string> route;
          string currentCity = end;

          while (!currentCity.empty()) {
               route.push_back(currentCity);
               currentCity = parent[currentCity];
          }

          reverse(route.begin(), route.end());

          printRoute(route);
          double distance = distanceMap[end];  // Total distance is the last recorded distance
          printRouteDetails(route, distance, duration / 1e6);  // Convert microseconds to seconds
     }
     else {
          cout << "No route found from " << start << " to " << end << "." << endl;
     }
}

// Helper function for iterative deepening depth-first search
void RouteFinder::iterativeDeepeningDFSHelper(const string& currentCity, const string& end,
     unordered_map<string, bool>& visited,
     unordered_map<string, string>& parent,
     unordered_map<string, double>& distanceMap,
     int depthLimit, bool& routeFound) {
     if (currentCity == end) {
          routeFound = true;
          return;
     }

     if (depthLimit <= 0) {
          return;
     }

     visited[currentCity] = true;

     for (const string& neighbor : adjacencyMap[currentCity]) {
          if (!visited[neighbor]) {
               parent[neighbor] = currentCity;

               // Accumulate distance
               distanceMap[neighbor] = distanceMap[currentCity] + calculateDistance(cityMap[currentCity], cityMap[neighbor]);

               iterativeDeepeningDFSHelper(neighbor, end, visited, parent, distanceMap, depthLimit - 1, routeFound);
          }
     }
}


// Function to find a route using depth-first search
void RouteFinder::depthFirstSearch(const string& start, const string& end) {
     auto startTime = chrono::high_resolution_clock::now();

     unordered_map<string, bool> visited;
     unordered_map<string, string> parent;
     unordered_map<string, double> distanceMap;  // New map to store distances

     bool routeFound = false;

     depthFirstSearchHelper(start, end, visited, parent, distanceMap, routeFound);

     auto endTime = chrono::high_resolution_clock::now();
     auto duration = chrono::duration_cast<chrono::microseconds>(endTime - startTime).count();

     if (routeFound) {
          vector<string> route;
          string currentCity = end;

          while (!currentCity.empty()) {
               route.push_back(currentCity);
               currentCity = parent[currentCity];
          }

          reverse(route.begin(), route.end());

          printRoute(route);
          double distance = distanceMap[end];  // Total distance is the last recorded distance
          printRouteDetails(route, distance, duration / 1e6);  // Convert microseconds to seconds
     }
     else {
          cout << "No route found from " << start << " to " << end << "." << endl;
     }
}

// Helper function for depth-first search
void RouteFinder::depthFirstSearchHelper(const string& currentCity, const string& end,
     unordered_map<string, bool>& visited,
     unordered_map<string, string>& parent,
     unordered_map<string, double>& distanceMap,
     bool& routeFound) {
     visited[currentCity] = true;

     if (currentCity == end) {
          routeFound = true;
          return;
     }

     for (const string& neighbor : adjacencyMap[currentCity]) {
          if (!visited[neighbor]) {
               parent[neighbor] = currentCity;

               // Accumulate distance
               distanceMap[neighbor] = distanceMap[currentCity] + calculateDistance(cityMap[currentCity], cityMap[neighbor]);

               depthFirstSearchHelper(neighbor, end, visited, parent, distanceMap, routeFound);
          }
     }
}



// Function to find a route using breadth-first search
void RouteFinder::breadthFirstSearch(const string& start, const string& end) {
     auto startTime = chrono::high_resolution_clock::now();

     unordered_map<string, bool> visited;
     unordered_map<string, string> parent;
     unordered_map<string, double> distanceMap;  // New map to store distances

     queue<string> q;

     q.push(start);
     visited[start] = true;
     distanceMap[start] = 0.0;  // Starting distance is 0

     bool routeFound = false;

     while (!q.empty()) {
          string currentCity = q.front();
          q.pop();

          if (currentCity == end) {
               routeFound = true;
               break;
          }

          for (const string& neighbor : adjacencyMap[currentCity]) {
               if (!visited[neighbor]) {
                    q.push(neighbor);
                    visited[neighbor] = true;
                    parent[neighbor] = currentCity;

                    // Accumulate distance
                    distanceMap[neighbor] = distanceMap[currentCity] + calculateDistance(cityMap[currentCity], cityMap[neighbor]);
               }
          }
     }

     auto endTime = chrono::high_resolution_clock::now();
     auto duration = chrono::duration_cast<chrono::microseconds>(endTime - startTime).count();

     if (routeFound) {
          vector<string> route;
          string currentCity = end;

          while (!currentCity.empty()) {
               route.push_back(currentCity);
               currentCity = parent[currentCity];
          }

          reverse(route.begin(), route.end());

          printRoute(route);
          double distance = distanceMap[end];  // Total distance is the last recorded distance
          printRouteDetails(route, distance, duration / 1e6);  // Convert microseconds to seconds
     }
     else {
          cout << "No route found from " << start << " to " << end << "." << endl;
     }
}

// Helper function to print the route
void RouteFinder::printRoute(const vector<string>& route) {
     cout << "Route: ";
     for (const string& city : route) {
          cout << city << " -> ";
     }
     cout << "Destination" << endl;
}

// Helper function to print route details
void RouteFinder::printRouteDetails(const vector<string>& route, double distance, double time) {
     cout << "Total distance: " << distance << " units" << endl;
     cout << "Total time: " << time << " seconds" << endl;
}

// Function to load cities from the CSV file
void RouteFinder::loadCities(const string& citiesFile) {
     ifstream file(citiesFile);
     if (!file.is_open()) {
          cerr << "Error opening cities file: " << citiesFile << endl;
          return;
     }

     string line;
     while (getline(file, line)) {
          stringstream ss(line);
          string cityName, latitude, longitude;

          getline(ss, cityName, ',');
          getline(ss, latitude, ',');
          getline(ss, longitude, ',');

          City city;
          city.name = cityName;
          city.latitude = stod(latitude);
          city.longitude = stod(longitude);

          cityMap[cityName] = city;
     }

     file.close();
}

// Function to load adjacencies from the text file
void RouteFinder::loadAdjacencies(const string& adjacenciesFile) {
     ifstream file(adjacenciesFile);
     if (!file.is_open()) {
          cerr << "Error opening adjacencies file: " << adjacenciesFile << endl;
          return;
     }

     string line;
     while (getline(file, line)) {
          stringstream ss(line);
          string city1, city2;

          ss >> city1 >> city2;

          // Adding bidirectional connections
          adjacencyMap[city1].push_back(city2);
          adjacencyMap[city2].push_back(city1);
     }

     file.close();
}

int main() {
     RouteFinder routeFinder;

     // Load data from files
     routeFinder.loadCities("coordinates.csv");
     routeFinder.loadAdjacencies("Adjacencies.txt");

     string start, end;
     int choice;

     do {
          cout << "Enter starting town: ";
          cin >> start;
          cout << "Enter ending town: ";
          cin >> end;

          cout << "Select search method:\n";
          cout << "1. Breadth-First Search\n";
          cout << "2. Depth-First Search\n";
          cout << "3. Iterative Deepening DFS\n";
          cout << "4. Best-First Search\n";
          cout << "5. A* Search\n";
          cout << "0. Exit\n";
          cout << "Enter choice: ";
          cin >> choice;

          switch (choice) {
          case 1:
               routeFinder.breadthFirstSearch(start, end);
               break;
          case 2:
               routeFinder.depthFirstSearch(start, end);
               break;
          case 3:
               routeFinder.iterativeDeepeningDFS(start, end);
               break;
          case 4:
               routeFinder.bestFirstSearch(start, end);
               break;
          case 5:
               routeFinder.aStarSearch(start, end);
               break;
          case 0:
               cout << "Exiting program.\n";
               break;
          default:
               cout << "Invalid choice. Please try again.\n";
          }

     } while (choice != 0);

     return 0;
}

