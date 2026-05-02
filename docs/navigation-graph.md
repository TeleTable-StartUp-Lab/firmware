# Navigation Graph

This firmware implements autonomous navigation as a small directed graph inside `src/app/navigation_controller.cpp`. The graph is currently compile-time data rather than a configurable map.

## Core Model

The navigation layer uses three main data structures from `NavigationController`:

- `GraphNode`
  Contains a stable node ID, a human-readable label, and the RFID UID associated with that location.
- `GraphEdge`
  Describes a directed connection from one node index to another and the `mapHeading` the robot should face while traversing that edge.
- `PlannedStep`
  Represents one executable step in a planned route: from-node, to-node, and the derived action (`STRAIGHT`, `TURN_RIGHT`, `TURN_LEFT`, or `REVERSE`).

The current graph is backed by:

- `GRAPH_NODES`
- `GRAPH_EDGES`
- `GRAPH_NODE_COUNT`
- `GRAPH_EDGE_COUNT`
- `MAX_PATH_STEPS`

The implementation also keeps `currentHeadingDegrees`, which is the robot's current belief about orientation. `0` degrees is treated as the Home-to-Kitchen direction.

## How Routing Works

When a route request arrives through HTTP or websocket control, `NavigationController::requestNavigation()`:

1. Validates the requested start and target node IDs.
2. Rejects the request if localization has been lost and Home reinitialization is required.
3. Uses the current localized node as the real start position.
4. Builds a path with `buildPath(...)`.
5. Converts each traversed edge into an action based on the difference between `mapHeading` and the robot's simulated heading.
6. Stores the resulting `PlannedStep` array and begins execution.

Path finding is a breadth-first search over the directed edge list. It is simple and predictable, but it only finds routes that exist in the current compile-time graph.

## How Localization Works

Localization is RFID-driven:

- each logical graph node has exactly one expected RFID UID
- `processRfid()` watches for new card reads
- a matching RFID UID updates the localized node
- if navigation is currently driving and the detected node matches the active step target, that step completes

The robot can lose localization when manual driving interrupts autonomous navigation. In that case:

- `loseLocalization()` clears current node and position
- `needsHomeReinitialization` becomes `true`
- the robot must return to the Home node before autonomous requests are accepted again

Home is special because the implementation resets heading to `0` there when reinitializing orientation.

## How Step Execution Works

Route execution is split into phases:

- `IDLE`
- `TURNING`
- `DRIVING`

`startStep()` chooses the next action:

- `STRAIGHT` starts forward drive
- `REVERSE` starts reverse drive
- `TURN_LEFT` or `TURN_RIGHT` starts an in-place turn

Turning relies on IMU gyro Z readings:

- accumulated turn degrees are integrated while the robot is turning
- when the target threshold is reached, heading is updated and the robot transitions into forward driving
- if the IMU is unavailable or the turn takes too long, navigation fails with an error

## Current Assumptions

- The node list is hard-coded in source.
- Each logical location has one RFID tag and one node entry.
- Edges are directed, so both directions must be added explicitly if travel is allowed both ways.
- Turn selection is based on heading differences, not a richer geometric model.
- The current route buffer is fixed by `MAX_PATH_STEPS`.
- The Home node anchors orientation reset behavior.

## How To Add More Nodes

To extend the autonomous map safely, update the graph in `src/app/navigation_controller.cpp` and keep the header constants in sync.

### 1. Add node constants

Add the new constants near the existing node definitions:

- node ID string
- display label string
- RFID UID string

Then append a new entry to `GRAPH_NODES`.

### 2. Update graph size constants

Increase the graph count constants so they match the actual array sizes:

- `NAV_GRAPH_NODE_COUNT`
- `NAV_GRAPH_EDGE_COUNT`

If longer routes are now possible, also increase:

- `MAX_PATH_STEPS` in `include/app/navigation_controller.h`

If `MAX_PATH_STEPS` is too small, route generation can fail or truncate valid paths conceptually even if the graph itself is correct.

### 3. Add directed edges

For each traversable corridor or connection, add a `GraphEdge` entry with:

- source node index
- destination node index
- `mapHeading` for travel along that edge

If the corridor can be traveled in both directions, add two edges. The reverse edge usually has a heading 180 degrees opposite the forward direction.

### 4. Verify heading consistency

The current turn planner assumes headings are coherent across the graph. Check that:

- the Home-facing reference is still valid
- every edge heading matches the robot's real-world orientation while moving through that segment
- left/right turns derived from heading deltas match physical behavior

If headings are inconsistent, the BFS path may still be valid as a graph, but the robot will choose the wrong turning action while executing it.

### 5. Verify RFID placement

Each node should have a unique RFID tag placed where step completion is physically meaningful. If tags are too early, too late, or reused between locations, localization and route progression will become unreliable.

### 6. Test the expanded graph

After changing the graph, test at least these cases:

- valid routes between old and new nodes
- multi-hop routes with several turns
- bidirectional travel on every new corridor
- rerouting from a localized node that is not Home
- manual override followed by Home reinitialization
- unknown RFID handling
- routes long enough to stress the new `MAX_PATH_STEPS`

## When The Current Design Becomes Limiting

The current implementation is fine for a small indoor graph, but it will become awkward if you need:

- non-90-degree turns or curved geometry
- very long paths that exceed `MAX_PATH_STEPS`
- many nodes and edges, where manual index management becomes error-prone
- richer localization than “latest RFID tag equals current node”
- dynamic map loading instead of recompiling the firmware

If the graph grows significantly, a good next step would be to move node and edge definitions into a more structured map representation and reduce reliance on compile-time fixed arrays.
