#include "search_run.h"
#include "wifi_manager.h"
#include "encoder.h"
#include "motor_control.h"
#include "turn.h"
#include "vl53l0x_v2.h"
#include "gyro.h"

namespace SearchRun {

static WiFiManager* wifi = nullptr;
static Encoder* enc = nullptr;
static MotorControl* motors = nullptr;

static const int SIZE = 16;                // 16x16 maze
static const float CELL_METERS = 0.18f;    // 18 cm cells
// ToF wall thresholds
static const float FRONT_THRESH_MM = 80.0f; // front wall threshold = 8 cm
static const float SIDE_THRESH_MM  = 70.0f;  // left/right wall threshold = 7 cm
static const int BASE_SPEED = 170;         // base forward speed matching moveforward.cpp

// Maze representation: walls per cell (N,E,S,W bits)
static uint8_t walls[SIZE][SIZE];   // bit0=N, bit1=E, bit2=S, bit3=W; 1=wall known, 0=open/unknown
static uint8_t known[SIZE][SIZE];   // bit0=N, bit1=E, bit2=S, bit3=W; 1=edge sensed (open or wall)
static bool visited[SIZE][SIZE];
static int dist[SIZE][SIZE];        // flood fill distances to goal

// Robot state
static int rx = 0, ry = 0;          // robot cell coordinates
static Dir rdir = NORTH;            // initial direction (assumption); can be changed before run if needed

// Goal discovery: start unknown, update when center found
static int goal_x = -1, goal_y = -1;  // Top-left of 2x2 goal, -1 = not found yet
static bool goal_found = false;

// Goal cells: center 2x2 (discovered dynamically)
static inline bool isGoal(int x, int y) {
    if (!goal_found) return false;  // Haven't found center yet
    return (x >= goal_x && x <= goal_x + 1) && (y >= goal_y && y <= goal_y + 1);
}

static inline void send(const String& s) {
    Serial.println(s);
    if (wifi && wifi->isConnected()) wifi->sendDataLn(s);
}

static inline int dx(Dir d) { return (d == EAST) - (d == WEST); }
static inline int dy(Dir d) { return (d == NORTH) - (d == SOUTH); }
static inline Dir turnLeft(Dir d) { return (Dir)((4 + d - 1) % 4); }
static inline Dir turnRight(Dir d) { return (Dir)((d + 1) % 4); }
static inline Dir turnBack(Dir d) { return (Dir)((d + 2) % 4); }

static inline uint8_t wallBit(Dir d) {
    switch (d) {
        case NORTH: return 1 << 0;
        case EAST:  return 1 << 1;
        case SOUTH: return 1 << 2;
        case WEST:  return 1 << 3;
    }
    return 0;
}

static inline uint8_t oppositeBit(Dir d) { return wallBit(turnBack(d)); }

// Mark a wall in current cell and mirror to neighbor
static void setWall(int x, int y, Dir d)
{
    if (x < 0 || y < 0 || x >= SIZE || y >= SIZE) return;
    walls[y][x] |= wallBit(d);
    int nx = x + dx(d);
    int ny = y + dy(d);
    if (nx >= 0 && ny >= 0 && nx < SIZE && ny < SIZE) {
        walls[ny][nx] |= oppositeBit(d);
    }
}

// Mark an edge as known (sensed), mirror to neighbor
static void setKnownEdge(int x, int y, Dir d)
{
    if (x < 0 || y < 0 || x >= SIZE || y >= SIZE) return;
    known[y][x] |= wallBit(d);
    int nx = x + dx(d);
    int ny = y + dy(d);
    if (nx >= 0 && ny >= 0 && nx < SIZE && ny < SIZE) {
        known[ny][nx] |= oppositeBit(d);
    }
}

// Manhattan distance heuristic to goal
static int manhattan(int x, int y) {
    if (!goal_found) {
        // No goal yet - use grid center as temporary heuristic for exploration
        int dx = abs(x - 7) + abs(x - 8);
        int dy = abs(y - 7) + abs(y - 8);
        return min(dx, dy) / 2;
    }
    // Goal found - use actual goal position
    int dx = min(abs(x - goal_x), abs(x - (goal_x + 1)));
    int dy = min(abs(y - goal_y), abs(y - (goal_y + 1)));
    return dx + dy;
}

// Detect center: look for 2x2 open area with minimal internal walls
// Micromouse center is typically 4 cells forming a square with openings
static void detectCenter() {
    if (goal_found) return;  // Already found
    
    // Scan for a 2x2 cluster where all 4 cells are accessible and connected
    for (int y = 1; y < SIZE - 2; ++y) {
        for (int x = 1; x < SIZE - 2; ++x) {
            // Check 2x2 block at (x,y), (x+1,y), (x,y+1), (x+1,y+1)
            // Center criteria: no walls between the 4 cells (internally connected)
            bool valid = true;
            
            // Check internal connectivity
            // (x,y) <-> (x+1,y): no wall on East of (x,y)
            if (walls[y][x] & wallBit(EAST)) valid = false;
            // (x,y) <-> (x,y+1): no wall on North of (x,y)
            if (walls[y][x] & wallBit(NORTH)) valid = false;
            // (x+1,y) <-> (x+1,y+1): no wall on North of (x+1,y)
            if (walls[y][x+1] & wallBit(NORTH)) valid = false;
            // (x,y+1) <-> (x+1,y+1): no wall on East of (x,y+1)
            if (walls[y+1][x] & wallBit(EAST)) valid = false;
            
            if (valid && visited[y][x] && visited[y][x+1] && visited[y+1][x] && visited[y+1][x+1]) {
                // Found a candidate 2x2 open area that we've explored
                goal_x = x;
                goal_y = y;
                goal_found = true;
                send(String("CENTER_FOUND|Goal at (") + goal_x + "," + goal_y + ") to (" + 
                     (goal_x+1) + "," + (goal_y+1) + ")");
                return;
            }
        }
    }
}

// A* search from current position to goal, considering only known walls
// Returns true if path exists, updates dist[] with g-cost from current cell
static bool computeAStarPath()
{
    const int INF = 1e9;
    for (int y = 0; y < SIZE; ++y)
        for (int x = 0; x < SIZE; ++x)
            dist[y][x] = INF;

    // Priority queue: [f-cost, x, y]
    static int pq_f[SIZE * SIZE], pq_x[SIZE * SIZE], pq_y[SIZE * SIZE];
    int pq_size = 0;
    
    // g-cost (actual distance from start)
    static int g_cost[SIZE][SIZE];
    for (int y = 0; y < SIZE; ++y)
        for (int x = 0; x < SIZE; ++x)
            g_cost[y][x] = INF;
    
    // Start from current robot position
    g_cost[ry][rx] = 0;
    dist[ry][rx] = 0;
    int f_cost = manhattan(rx, ry);
    pq_f[pq_size] = f_cost; pq_x[pq_size] = rx; pq_y[pq_size] = ry;
    pq_size++;
    
    bool foundGoal = false;
    
    while (pq_size > 0) {
        // Extract min f-cost (simple linear search for small mazes)
        int minIdx = 0;
        for (int i = 1; i < pq_size; ++i) {
            if (pq_f[i] < pq_f[minIdx]) minIdx = i;
        }
        int f = pq_f[minIdx];
        int x = pq_x[minIdx];
        int y = pq_y[minIdx];
        // Remove from queue
        pq_f[minIdx] = pq_f[pq_size - 1];
        pq_x[minIdx] = pq_x[pq_size - 1];
        pq_y[minIdx] = pq_y[pq_size - 1];
        pq_size--;
        
        // Check if goal reached
        if (isGoal(x, y)) {
            foundGoal = true;
            break;
        }
        
        int current_g = g_cost[y][x];
        
        // Explore neighbors (only if no known wall)
        Dir directions[4] = {NORTH, EAST, SOUTH, WEST};
        for (int i = 0; i < 4; ++i) {
            Dir d = directions[i];
            if (walls[y][x] & wallBit(d)) continue;  // Skip known walls
            
            int nx = x + dx(d);
            int ny = y + dy(d);
            if (nx < 0 || ny < 0 || nx >= SIZE || ny >= SIZE) continue;  // Out of bounds
            
            int tentative_g = current_g + 1;
            if (tentative_g < g_cost[ny][nx]) {
                g_cost[ny][nx] = tentative_g;
                dist[ny][nx] = tentative_g;  // Store g-cost for gradient descent
                int h = manhattan(nx, ny);
                int new_f = tentative_g + h;
                
                // Add to queue
                pq_f[pq_size] = new_f;
                pq_x[pq_size] = nx;
                pq_y[pq_size] = ny;
                pq_size++;
            }
        }
    }
    
    return foundGoal;
}

// Sense walls using ToF readings given current robot direction
static float lastFront = 0, lastRight = 0, lastLeft = 0;

static void senseWalls()
{
    // read sensors (front, right, left)
    if (!readToF()) return;
    float front = tof_distance[TOF_FRONT];
    float right = tof_distance[TOF_RIGHT];
    float left  = tof_distance[TOF_LEFT];
    lastFront = front; lastRight = right; lastLeft = left;

    // front
    setKnownEdge(rx, ry, rdir);
    if (front > 0 && front <= FRONT_THRESH_MM) setWall(rx, ry, rdir);
    // right
    setKnownEdge(rx, ry, turnRight(rdir));
    if (right > 0 && right <= SIDE_THRESH_MM) setWall(rx, ry, turnRight(rdir));
    // left
    setKnownEdge(rx, ry, turnLeft(rdir));
    if (left > 0 && left <= SIDE_THRESH_MM) setWall(rx, ry, turnLeft(rdir));

    String dirName = "?";
    if (rdir == NORTH) dirName = "N";
    else if (rdir == EAST) dirName = "E";
    else if (rdir == SOUTH) dirName = "S";
    else if (rdir == WEST) dirName = "W";
    
    String s = "|ToF_F:" + String(front, 1) + "mm|ToF_R:" + String(right, 1) + "mm|ToF_L:" + String(left, 1) + "mm";
               
    send(s);
}

// Turn robot to desired direction (minimal turns)
static void orientTo(Dir target)
{
    int diff = ((int)target - (int)rdir + 4) % 4;
    if (diff == 0) return;
    if (diff == 1) { turnRight90(); rdir = target; }
    else if (diff == 3) { turnLeft90(); rdir = target; }
    else { turn180(); rdir = target; }

    // After orienting, capture the current yaw as the forward reference
    if (gyro_ok) {
        updateGyro();
        // Record the yaw we want to maintain during the next forward step
        // This mirrors MoveForward's approach of maintaining initialYaw
        // by sampling immediately after orientation.
    }
}

// Move forward one cell
// Gyro-assisted alignment after forward move
static float yawTolerance = 1.0f;

static void alignToYaw(float targetYaw)
{
    if (!gyro_ok) return;

    updateGyro();
    float yawError = targetYaw - currentYaw;
    while (yawError > 180.0f) yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;

    if (abs(yawError) < yawTolerance) {
        motors->stop();
        delay(100);
        return;
    }

    int turnSpeed = BASE_SPEED;
    while (abs(yawError) > yawTolerance) {
        updateGyro();
        yawError = targetYaw - currentYaw;
        while (yawError > 180.0f) yawError -= 360.0f;
        while (yawError < -180.0f) yawError += 360.0f;

        if (yawError > 0) {
            motors->setMotorASpeed(turnSpeed);
            motors->setMotorBSpeed(-turnSpeed);
        } else {
            motors->setMotorASpeed(-turnSpeed);
            motors->setMotorBSpeed(turnSpeed);
        }

        String output = String("Yaw_:") + String(currentYaw) + "|Target:" + String(targetYaw) + "|Err:" + String(yawError);
        send(output);
        delay(20);
    }

    motors->stop();
    delay(200);
}

// Move forward one cell using the same encoder+PID loop as moveforward.cpp
static void stepForward()
{
    // Determine target yaw reference at start of forward step
    float targetYaw = 0.0f;
    if (gyro_ok) {
        updateGyro();
        targetYaw = currentYaw;
    }

    // Reset encoder distances
    enc->reset(true);

    while (true) {
        // Update gyro (optional)
        if (gyro_ok) updateGyro();

        // Drive forward with straight-line PID correction
        motors->moveForward(BASE_SPEED);
        motors->updateStraightLineControl();

        // Read encoder distances
        float leftDist = enc->getDistance1();
        float rightDist = enc->getDistance2();
        float avgDist = (abs(leftDist) + abs(rightDist)) / 2.0f;
        
        // Read ToF (front, right, left)
        readToF();
        float frontToF = tof_distance[TOF_FRONT];
        float rightToF = tof_distance[TOF_RIGHT];
        float leftToF  = tof_distance[TOF_LEFT];

        // Telemetry with cell position, direction, and all ToFs
        String dirName = "?";
        if (rdir == NORTH) dirName = "N";
        else if (rdir == EAST) dirName = "E";
        else if (rdir == SOUTH) dirName = "S";
        else if (rdir == WEST) dirName = "W";
        
        String output = String("Step|Cell(") + rx + "," + ry + ")|Dir:" + dirName +
                        "|Dist:" + String(avgDist * 100.0f, 1) + "cm|ToF_F:" + String(frontToF, 1) +
                        "mm|R:" + String(rightToF, 1) + "mm|L:" + String(leftToF, 1) + "mm";
        send(output);

        // Stop when we reach one cell distance
        if (avgDist >= CELL_METERS) {
            motors->stop();
            delay(100);
            break;
        }

        delay(20);
    }

    // Post-move yaw alignment similar to MoveForward
    if (gyro_ok) {
        updateGyro();
        float yawError = targetYaw - currentYaw;
        while (yawError > 180.0f) yawError -= 360.0f;
        while (yawError < -180.0f) yawError += 360.0f;
        if (abs(yawError) > yawTolerance) {
            alignToYaw(targetYaw);
        }
    }

    // Update grid coordinates
    rx += dx(rdir);
    ry += dy(rdir);
    if (rx < 0) rx = 0; if (ry < 0) ry = 0; if (rx >= SIZE) rx = SIZE - 1; if (ry >= SIZE) ry = SIZE - 1;
}

// Ensure the path in current facing direction is clear. If unknown, probe with ToF.
// Returns true if clear (no wall), false if blocked and wall map updated.
static bool ensureFrontClear()
{
    if (!readToF()) return false; // fail safe: avoid moving when sensors not available
    float front = tof_distance[TOF_FRONT];
    if (front > 0 && front <= FRONT_THRESH_MM) {
        setWall(rx, ry, rdir);
        send(String("Block|C(") + rx + "," + ry + ")|Dir:" + (int)rdir + "|F:" + String(front,0));
        return false;
    }
    return true;
}

// Choose next direction using A* path or explore unknown edges
static Dir chooseNext()
{
    int best = 1e9;
    Dir bestDir = NORTH;
    bool found = false;
    
    // Directions in preference order: forward, left, right, back
    Dir order[4] = { rdir, turnLeft(rdir), turnRight(rdir), turnBack(rdir) };
    
    // First try: follow A* gradient (lowest g-cost neighbor)
    for (int i = 0; i < 4; ++i) {
        Dir d = order[i];
        int nx = rx + dx(d);
        int ny = ry + dy(d);
        if (nx < 0 || ny < 0 || nx >= SIZE || ny >= SIZE) continue;
        if (walls[ry][rx] & wallBit(d)) continue;  // Skip known walls
        
        int cand = dist[ny][nx];
        if (cand < best) { 
            best = cand; 
            bestDir = d;
            found = true;
        }
    }
    
    // If no valid A* path, explore nearest unknown edge
    if (!found || best >= 1e9) {
        send("Exploring unknown edges");
        for (int i = 0; i < 4; ++i) {
            Dir d = order[i];
            int nx = rx + dx(d);
            int ny = ry + dy(d);
            if (nx < 0 || ny < 0 || nx >= SIZE || ny >= SIZE) continue;
            if (walls[ry][rx] & wallBit(d)) continue;  // Skip known walls
            
            // Prioritize unknown edges
            if (!(known[ry][rx] & wallBit(d))) {
                bestDir = d;
                found = true;
                send(String("Exploring unknown edge in direction:") + (int)d);
                break;
            }
        }
    }
    
    // Last resort: any open direction
    if (!found) {
        for (int i = 0; i < 4; ++i) {
            Dir d = (Dir)i;
            int nx = rx + dx(d);
            int ny = ry + dy(d);
            if (nx < 0 || ny < 0 || nx >= SIZE || ny >= SIZE) continue;
            if (walls[ry][rx] & wallBit(d)) continue;
            bestDir = d;
            found = true;
            send(String("Last resort|Dir:") + (int)d);
            break;
        }
    }
    
    if (!found) {
        send(String("TRAPPED|Cell(") + rx + "," + ry + ")|All neighbors blocked");
    }
    
    return bestDir;
}

void begin(WiFiManager* w, Encoder* e, MotorControl* m, int startX, int startY, Dir startDir)
{
    wifi = w; enc = e; motors = m;
    // reset maps
    for (int y = 0; y < SIZE; ++y) {
        for (int x = 0; x < SIZE; ++x) {
            walls[y][x] = 0; known[y][x] = 0; visited[y][x] = false; dist[y][x] = 0;
        }
    }
    rx = startX; ry = startY; rdir = startDir;
    // Clamp to grid bounds
    if (rx < 0) rx = 0; if (rx >= SIZE) rx = SIZE - 1;
    if (ry < 0) ry = 0; if (ry >= SIZE) ry = SIZE - 1;
    
    // Reset goal discovery
    goal_x = -1; goal_y = -1; goal_found = false;
    
    String dirStr = "?";
    if (startDir == NORTH) dirStr = "N";
    else if (startDir == EAST) dirStr = "E";
    else if (startDir == SOUTH) dirStr = "S";
    else if (startDir == WEST) dirStr = "W";
    
    send(String("SearchRun initialized at (") + rx + "," + ry + ") facing " + dirStr);
}

void run()
{
    // on-the-fly flood fill to center
    int steps = 0;
    while (!isGoal(rx, ry) && steps < 512) {
        ++steps;
        
        // Update WiFi client connection
        if (wifi) wifi->update();
        
        // Sense walls at current cell
        senseWalls();
        visited[ry][rx] = true;
        
        // Try to detect center after visiting each new cell
        detectCenter();

        // Recompute A* path using current knowledge
        bool pathExists = computeAStarPath();

        // Choose next direction by lowest neighbor distance, verify unknown edges using relevant ToF
        while (true) {
            // Refresh WiFi client
            if (wifi) wifi->update();
            
            Dir next = chooseNext();
            
            String currDir = "?", nextDir = "?";
            if (rdir == NORTH) currDir = "N"; else if (rdir == EAST) currDir = "E";
            else if (rdir == SOUTH) currDir = "S"; else if (rdir == WEST) currDir = "W";
            if (next == NORTH) nextDir = "N"; else if (next == EAST) nextDir = "E";
            else if (next == SOUTH) nextDir = "S"; else if (next == WEST) nextDir = "W";
            
            send(String("Plan|Cell(") + rx + "," + ry + ")|Dir:" + currDir + "->" + nextDir +
                 "|Dist:" + dist[ry][rx] + "|ToF_F:" + String(lastFront, 1) +
                 "mm|R:" + String(lastRight, 1) + "mm|L:" + String(lastLeft, 1) + "mm");

            // CRITICAL: Check if this direction has a known wall - if so, replan
            if (walls[ry][rx] & wallBit(next)) {
                send(String("BLOCKED|Cell(") + rx + "," + ry + ")|Dir:" + nextDir + " is blocked by known wall");
                computeAStarPath();
                continue;  // Re-plan, don't try to move
            }

            // If edge already unknown, validate with ToF before moving
            bool edgeKnown = (known[ry][rx] & wallBit(next)) != 0;
            if (!edgeKnown) {
                // Use current orientation ToFs to validate without turning when possible
                bool clear = false;
                if (next == rdir) {
                    clear = (lastFront == 0) || (lastFront > FRONT_THRESH_MM);
                    if (!clear) setWall(rx, ry, rdir);
                } else if (next == turnLeft(rdir)) {
                    // For left turn, check left ToF before committing
                    clear = (lastLeft == 0) || (lastLeft > SIDE_THRESH_MM);
                    if (!clear) {
                        setWall(rx, ry, turnLeft(rdir));
                        send(String("Left blocked|C(") + rx + "," + ry + ")|L:" + String(lastLeft, 1) + "mm");
                    }
                } else if (next == turnRight(rdir)) {
                    // For right turn, check right ToF before committing
                    clear = (lastRight == 0) || (lastRight > SIDE_THRESH_MM);
                    if (!clear) {
                        setWall(rx, ry, turnRight(rdir));
                        send(String("Right blocked|C(") + rx + "," + ry + ")|R:" + String(lastRight, 1) + "mm");
                    }
                } else { // back: no side sensor for behind; orient and probe front
                    orientTo(next);
                    if (!ensureFrontClear()) { computeAStarPath(); continue; }
                    // Path clear after probe
                    stepForward();
                    break;
                }

                if (!clear) { computeAStarPath(); continue; }
                // mark edge as known open
                setKnownEdge(rx, ry, next);
            }

            // Turn toward next and move one cell
            orientTo(next);
            stepForward();
            break;
        }
    }

    if (goal_found) {
        send(String("SUCCESS|Reached center at (") + rx + "," + ry + ") in " + steps + " steps");
    } else {
        send(String("SEARCH_END|Explored " + String(steps) + " steps but center not found"));
    }
}

} // namespace SearchRun
