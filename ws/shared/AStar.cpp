#include "AStar.h"

MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic){

    // initalize the graph
    std::shared_ptr<amp::Graph<double>> graph = problem.graph;

    // create a GraphSearchResult to store in
    MyAStarAlgo::GraphSearchResult result;

    // graph->print("Current Graph");

    // Priority queue
    std::set<std::pair<double, amp::Node>> queue;

    // Create a map to store the cost of each node, will be sum of heuristic and edge cost
    std::map<amp::Node, double> cost_map;

    // Create a map to store the parent node of each visited node
    std::map<amp::Node, amp::Node> parent_map;

    // Add the start node to the queue and cost map
    queue.insert({heuristic.operator()(problem.init_node), problem.init_node});
    cost_map[problem.init_node] = 0;
    int count = 1;
    while (!queue.empty()){
        count++;
        // Get the node with the lowest cost
        amp::Node curr_node = queue.begin()->second;
        double curr_cost = queue.begin()->first;
        queue.erase(queue.begin());

        // Check if we've reached the goal node
        if (curr_node == problem.goal_node) {
            // Backtrack the path from the goal node to the start node
            // std::cout << "Goal found!" << std::endl;
            // Clear node_path, will override each time goal found
            result.node_path.clear();
            result.node_path.push_back(curr_node);
            while (curr_node != problem.init_node) {
                curr_node = parent_map[curr_node];
                result.node_path.push_back(curr_node);
            }
            std::reverse(result.node_path.begin(), result.node_path.end());

            // Set the result node cost
            result.path_cost = curr_cost;
            result.success = true;
            break;
        }

        // Expand the current node's neighbors
        std::vector<amp::Node> neighbors = graph->children(curr_node);
        std::vector<double> edge_costs = graph->outgoingEdges(curr_node);
        for (int i = 0; i < neighbors.size(); i++) {
            amp::Node neighbor = neighbors[i];
            double heuristic_cost = heuristic.operator()(neighbor); // only current heuristic
            double total_cost = curr_cost + edge_costs[i] + heuristic_cost - heuristic.operator()(curr_node);

            // Check if we've already visited this node with a lower cost
            if (cost_map.find(neighbor) != cost_map.end() && cost_map.find(neighbor)->second <= total_cost) {
                continue;
            }

            // output information
            // std::cout << "Added Node: " << neighbor << " with cost: " << total_cost << " and parent: " << curr_node << std::endl;

            // Update the cost map and parent map
            cost_map[neighbor] = total_cost;
            parent_map[neighbor] = curr_node;

            // Add the neighbor to the queue
            queue.insert({total_cost, neighbor});
        }
    }

    // Check if a path was found
    if (!result.success) {
        // return empty path
        result.node_path.clear();
        return result;
    }

    // std::cout << "Number of iterations: " << count << std::endl;
    return result;
}