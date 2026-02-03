```
// Main control loop
function navigate_robot(graph, start_node, goal_node):
    // Get shortest path using Dijkstra
    total_ticks, path = dijkstra(graph, start_node, goal_node)

    if path is null:
        error("No path exists from start to goal")
        return

    current_node = start_node
    path_index = 0

    while current_node != goal_node:
        // Read RFID to verify position
        detected_node = read_rfid_tag()

        if detected_node != current_node:
            // Robot is lost - recompute path from actual position
            print("Position mismatch! Expected:", current_node, "Detected:", detected_node)
            current_node = detected_node
            total_ticks, path = dijkstra(graph, current_node, goal_node)
            path_index = 0
            continue

        // Get next node in path
        path_index += 1
        next_node = path[path_index]

        // Look up edge weight (ticks needed)
        ticks_to_next = get_edge_weight(graph, current_node, next_node)

        // Reposition robot to face next node
        reposition_robot(current_node, next_node)

        // Drive to next node
        drive_for_ticks(ticks_to_next)

        // Update current position
        current_node = next_node

    // Verify we reached goal
    final_check = read_rfid_tag()
    if final_check == goal_node:
        print("Successfully reached goal!")
    else:
        print("Error: At wrong node", final_check)


// Dijkstra's algorithm
function dijkstra(graph, start, goal):
    priority_queue = new MinHeap()
    priority_queue.push((0, start, [start]))
    visited = new Set()

    while priority_queue is not empty:
        (cost, node, path) = priority_queue.pop()

        if node in visited:
            continue

        visited.add(node)

        if node == goal:
            return (cost, path)

        for each (neighbor, ticks) in graph[node]:
            if neighbor not in visited:
                new_cost = cost + ticks
                new_path = path + [neighbor]
                priority_queue.push((new_cost, neighbor, new_path))

    return (null, null)  // No path found


// Helper: Get edge weight between two nodes
function get_edge_weight(graph, from_node, to_node):
    for each (neighbor, ticks) in graph[from_node]:
        if neighbor == to_node:
            return ticks

    error("No edge exists between nodes")
    return null


// Hardware interface stubs (you implement these)
function read_rfid_tag():
    // Read RFID sensor
    // Return node ID as string/int
    return node_id

function reposition_robot(current_node, next_node):
    // Calculate angle/direction to next node
    // Rotate robot to face correct direction
    // This depends on your physical setup
    pass

function drive_for_ticks(ticks):
    // Command motors to drive forward
    // Wait for specified number of encoder ticks
    // Stop motors
    pass
```

Key features:

- **RFID verification** at each node before moving
- **Automatic re-routing** if robot is mispositioned
- **Repositioning** before each drive segment
- **Tick-based driving** using your known weights

You'll need to implement the three hardware functions (`read_rfid_tag`, `reposition_robot`, `drive_for_ticks`) based on your actual robot platform.
