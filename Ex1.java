import java.io.*;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;


public class Ex1 {
    public static void main(String[] args) {
        try {
            InputHandler inputHandler = new InputHandler("input.txt");
            Board initialBoard = inputHandler.getInitialBoard();
            Board goalBoard = inputHandler.getGoalBoard();
            String algorithm = inputHandler.getAlgorithm();
            boolean withTime = inputHandler.isWithTime();
            boolean withOpenList = inputHandler.isWithOpenList();

            SearchSolver solver = new SearchSolver(initialBoard, goalBoard, algorithm, withTime, withOpenList);
            solver.solve();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

// Class to handle input reading
class InputHandler {
    private String algorithm;
    private boolean withTime;
    private boolean withOpenList;
    private Board initialBoard;
    private Board goalBoard;

    public InputHandler(String filename) throws IOException {
        BufferedReader br = new BufferedReader(new FileReader(filename));
        this.algorithm = br.readLine().trim();
        this.withTime = br.readLine().trim().equals("with time");
        this.withOpenList = br.readLine().trim().equals("open with");

        String[][] initial = new String[3][3];
        for (int i = 0; i < 3; i++) initial[i] = br.readLine().split(",");
        this.initialBoard = new Board(initial);

        br.readLine(); // Skip "Goal state:" line
        String[][] goal = new String[3][3];
        for (int i = 0; i < 3; i++) goal[i] = br.readLine().split(",");
        this.goalBoard = new Board(goal);
    }

    public String getAlgorithm() { return algorithm; }
    public boolean isWithTime() { return withTime; }
    public boolean isWithOpenList() { return withOpenList; }
    public Board getInitialBoard() { return initialBoard; }
    public Board getGoalBoard() { return goalBoard; }
}


// Class to represent a board state
class Board {
    private String[][] board;
    private static final Map<String, Integer> COST_MAP = new HashMap<String, Integer>() {{
        put("B", 1); // Blue marble cost
        put("G", 3); // Green marble cost
        put("R", 10); // Red marble cost
    }};

    public Board(String[][] board) {
        this.board = new String[3][3];
        for (int i = 0; i < 3; i++) {
            System.arraycopy(board[i], 0, this.board[i], 0, 3);
        }
    }

    public static Map<String, Integer> getCostMap() {
        return COST_MAP;
    }

    public List<Action> getPossibleActions() {
        List<Action> actions = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (board[i][j].equals("_")) {
                    checkAndAddAction(actions, i, j, (i + 1) % 3, j);       // Down
                    checkAndAddAction(actions, i, j, (i - 1 + 3) % 3, j);   // Up
                    checkAndAddAction(actions, i, j, i, (j + 1) % 3);       // Right
                    checkAndAddAction(actions, i, j, i, (j - 1 + 3) % 3);   // Left
                }
            }
        }
        return actions;
    }

    private void checkAndAddAction(List<Action> actions, int blankX, int blankY, int marbleX, int marbleY) {
        if (!board[marbleX][marbleY].equals("_") && !board[marbleX][marbleY].equals("X")) {
            int cost = COST_MAP.getOrDefault(board[marbleX][marbleY], 0);
            actions.add(new Action(marbleX, marbleY, blankX, blankY, board[marbleX][marbleY], cost));
        }
    }

    public Board applyAction(Action action) {
        String[][] newBoard = new String[3][3];
        for (int i = 0; i < 3; i++) {
            System.arraycopy(board[i], 0, newBoard[i], 0, 3);
        }
        newBoard[action.toX][action.toY] = board[action.fromX][action.fromY];
        newBoard[action.fromX][action.fromY] = "_";
        return new Board(newBoard);
    }

    public String[][] getBoard() {
        return board;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Board board1 = (Board) o;
        return Arrays.deepEquals(board, board1.board);
    }

    @Override
    public int hashCode() {
        return Arrays.deepHashCode(board);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < board.length; i++) {
            for (int j = 0; j < board[i].length; j++) {
                sb.append(board[i][j]);
                if (j < board[i].length - 1) {
                    sb.append(" "); // Add space between elements
                }
            }
            sb.append("\n"); // Add a new line after each row
        }
        return sb.toString();
    }

}

class Action {
    int fromX, fromY, toX, toY, cost;
    String color;

    public Action(int fromX, int fromY, int toX, int toY, String color, int cost) {
        this.fromX = fromX;
        this.fromY = fromY;
        this.toX = toX;
        this.toY = toY;
        this.color = color;
        this.cost = cost;
    }

    @Override
    public String toString() {
        return String.format("(%d,%d):%s:(%d,%d)", fromX + 1, fromY + 1, color, toX + 1, toY + 1);
    }
}

class SearchSolver {
    private Board initialBoard, goalBoard;
    private String algorithm;
    private boolean withTime, withOpenList;
    private int createdStates; 
    private int cost;

    public SearchSolver(Board initialBoard, Board goalBoard, String algorithm, boolean withTime, boolean withOpenList) {
        this.initialBoard = initialBoard;
        this.goalBoard = goalBoard;
        this.algorithm = algorithm;
        this.withTime = withTime;
        this.withOpenList = withOpenList;
    }

    public void solve() {
        if (algorithm.equals("BFS")) bfs();
        else if (algorithm.equals("A*")) aStar();
        else if (algorithm.equals("IDA*")) idaStar();
        else if(algorithm.equals("DFBnB")) dfbnb();
        else if (algorithm.equals("DFID")) dfid(); 
    }


    ///////////////////////////BFS/////////////////
    private void bfs() {
        long startTime = System.nanoTime();
        Queue<Node> open = new LinkedList<>();
        Set<Board> closed = new HashSet<>();
        int expandedCount = 0;
    
        open.add(new Node(initialBoard, 0, "", 0)); 
    
        while (!open.isEmpty()) {
            if (withOpenList) {
                System.out.println("Open list:");
                for (Node node : open) {
                    System.out.println(node.state); // Print the state of each node
                }
            }
            Node current = open.poll();
            expandedCount++;
    
            if (closed.contains(current.state)) continue;
            closed.add(current.state);
    
            // Goal check
            if (current.state.equals(goalBoard)) {
                String completePath = current.path.endsWith("--")
                        ? current.path.substring(0, current.path.length() - 2)
                        : current.path;
                writeOutput(completePath, expandedCount, current.g, System.nanoTime() - startTime);
                return;
            }
    
            // Expand current state
            for (Action action : current.state.getPossibleActions()) {
                Board nextState = current.state.applyAction(action);
                if (!closed.contains(nextState)) {
                    open.add(new Node(nextState, current.g + action.cost, current.path + action.toString() + "--", 0));
                }
            }
            if (withOpenList) printOpenList(open);
        }
    
        writeOutput("no path", expandedCount, Integer.MAX_VALUE, System.nanoTime() - startTime);
    }
    
    
    ////////////////////A*////////////////////

    private void aStar() {
        long startTime = System.nanoTime();
    
        
        PriorityQueue<Node> openQueue = new PriorityQueue<>((a, b) -> {
            if (a.f == b.f) return Integer.compare(a.g, b.g); // Tie-breaking on g-cost
            return Integer.compare(a.f, b.f);
        });
        Map<Board, Node> openMap = new HashMap<>(); // Hash table for open list
        Set<Board> closedSet = new HashSet<>();    // Set for closed list
        int expandedCount = 0;
    
        // Add the start node
        Node startNode = new Node(initialBoard, 0, "", heuristic(initialBoard));
        openQueue.add(startNode);
        openMap.put(initialBoard, startNode);
    
        while (!openQueue.isEmpty()) {
            // Print open list if "with open"
            if (withOpenList) {
             System.out.println("Open list:");
                for (Node node : openQueue) {
                    System.out.println(node.state); // Print the state of each node
                }
            }
            // Retrieve and remove the best node
            Node current = openQueue.poll();
            openMap.remove(current.state);
    
            // Goal check
            if (current.state.equals(goalBoard)) {
                // Format and clean the path
                String finalPath = current.path.endsWith("--") 
                    ? current.path.substring(0, current.path.length() - 2) 
                    : current.path;
                writeOutput(finalPath, expandedCount, current.g, System.nanoTime() - startTime);
                return;
            }
    
            // Mark current node as expanded
            expandedCount++;
            closedSet.add(current.state);
    
            // Expand the current node
            for (Action action : current.state.getPossibleActions()) {
                Board nextState = current.state.applyAction(action);
                int gCost = current.g + action.cost;
                int hCost = heuristic(nextState);
                int fCost = gCost + hCost;
    
                // Skip if in closed set
                if (closedSet.contains(nextState)) continue;
    
                // Check if in open list
                if (openMap.containsKey(nextState)) {
                    Node existingNode = openMap.get(nextState);
                    if (gCost < existingNode.g) {
                        // Update the existing node
                        openQueue.remove(existingNode);
                        existingNode.g = gCost;
                        existingNode.f = fCost;
                        existingNode.path = current.path + action.toString() + "--";
                        openQueue.add(existingNode);
                    }
                } else {
                    // Add the new node
                    Node newNode = new Node(nextState, gCost, current.path + action.toString() + "--", fCost);
                    openQueue.add(newNode);
                    openMap.put(nextState, newNode);
                }
            }
            if (withOpenList) printOpenList(openQueue);
        }
    
        // If no solution is found
        writeOutput("no path", expandedCount, Integer.MAX_VALUE, System.nanoTime() - startTime);
    }
    
    
    
    
    
///////////////////////////////////////////IDA*///////////////////////

private void idaStar() {
    long startTime = System.nanoTime();
    int threshold = heuristic(initialBoard); // Initial threshold
    int expandedCount = 0;

    while (threshold != Integer.MAX_VALUE) {
        int nextThreshold = Integer.MAX_VALUE;
        Stack<Node> stack = new Stack<>();
        Hashtable<String, Node> openList = new Hashtable<>();
        Hashtable<String, Node> outSet = new Hashtable<>();

        Node startNode = new Node(initialBoard, 0, "", heuristic(initialBoard));
        stack.push(startNode);
        openList.put(startNode.state.toString(), startNode);
        ++expandedCount;

        while (!stack.isEmpty()) {
            if (withOpenList) {
                System.out.println("Open list:");
                for (Node node : stack) {
                    System.out.println(node.state); // Print the state of each node
                }
            }

            Node current = stack.pop();

            // Check if already processed
            if (outSet.containsKey(current.state.toString())) {
                openList.remove(current.state.toString());
                continue;
            }

            outSet.put(current.state.toString(), current);
            stack.push(current);

            // Goal check
            if (current.state.equals(goalBoard)) {
                String completePath = current.path.endsWith("--")
                        ? current.path.substring(0, current.path.length() - 2)
                        : current.path;
                writeOutput(completePath, expandedCount, current.g, System.nanoTime() - startTime);
                return;
            }

            // Generate successors
            List<Action> actions = current.state.getPossibleActions();
            List<Node> successors = new ArrayList<>();
            for (Action action : actions) {
                Board nextState = current.state.applyAction(action);
                int gCost = current.g + action.cost;
                int fCost = gCost + heuristic(nextState);

                Node successor = new Node(nextState, gCost, current.path + action.toString() + "--", fCost);
                ++expandedCount;

                if (fCost > threshold) {
                    nextThreshold = Math.min(nextThreshold, fCost); // Update threshold
                } else if (!openList.containsKey(nextState.toString())) {
                    successors.add(successor);
                }
            }

            
            successors.sort((a, b) -> {
                if (a.f != b.f) return Integer.compare(a.f, b.f);
                return Integer.compare(a.g, b.g);
            });

            // Push successors onto the stack
            for (Node successor : successors) {
                stack.push(successor);
                openList.put(successor.state.toString(), successor);
            }
        }

        threshold = nextThreshold; // Update the threshold
    }

    writeOutput("No path", expandedCount, Integer.MAX_VALUE, System.nanoTime() - startTime);
}





    
    
    
    
////////////////////////////////////dfbnb/////////////////////////

    private void dfbnb() {
        long startTime = System.nanoTime();
    
        Stack<Node> stack = new Stack<>();
        Map<String, Node> openList = new HashMap<>();
        Set<String> outSet = new HashSet<>();
        AtomicInteger threshold = new AtomicInteger(Integer.MAX_VALUE);
        int expandedCount = 0;
    
        Node startNode = new Node(initialBoard, 0, "", heuristic(initialBoard));
        stack.push(startNode);
        openList.put(startNode.state.toString(), startNode);
    
        Node bestSolution = null;
    
        while (!stack.isEmpty()) {
            if (withOpenList) { // Print open list
                System.out.println("Open list:");
                for (Node node : stack) {
                    System.out.println(node.state);
                }
            }
            Node current = stack.pop();
    
            // Loop avoidance: if already processed,remove it
            if (outSet.contains(current.state.toString())) {
                openList.remove(current.state.toString());
                continue;
            }
    
            // Mark current node as being processed
            outSet.add(current.state.toString());
    
            // Goal check
            if (current.state.equals(goalBoard)) {
                if (current.f < threshold.get()) {
                    threshold.set(current.f);
                    bestSolution = current;
    
                    
                    stack.removeIf(node -> node.f >= threshold.get());
                }
                continue;
            }
    
            expandedCount++;
    
            // Generate successors
            List<Action> actions = current.state.getPossibleActions();
            List<Node> successors = new ArrayList<>();
            for (Action action : actions) {
                Board nextState = current.state.applyAction(action);
                int gCost = current.g + action.cost;
                int fCost = gCost + heuristic(nextState);
    
                Node successor = new Node(nextState, gCost, current.path + action.toString() + "--", fCost);
                successors.add(successor);
            }
    
            
            successors.sort((a, b) -> a.f == b.f ? Integer.compare(a.g, b.g) : Integer.compare(a.f, b.f));
    
            for (Iterator<Node> iterator = successors.iterator(); iterator.hasNext(); ) {
                Node successor = iterator.next();
    
                // Prune successors exceeding the threshold
                if (successor.f >= threshold.get()) {
                    iterator.remove();
                    continue;
                }
    
                // Check for loops or redundant states
                if (outSet.contains(successor.state.toString())) {
                    iterator.remove();
                } else if (openList.containsKey(successor.state.toString())) {
                    Node existing = openList.get(successor.state.toString());
                    if (existing.f <= successor.f) {
                        iterator.remove();
                    } else {
                        stack.remove(existing);
                        openList.remove(existing.state.toString());
                    }
                }
            }
    
            // Reverse the order for stack-based processing
            Collections.reverse(successors);
            for (Node successor : successors) {
                stack.push(successor);
                openList.put(successor.state.toString(), successor);
            }
        }
    
        // Output the best solution
        if (bestSolution != null) {
            String completePath = bestSolution.path.endsWith("--")
                ? bestSolution.path.substring(0, bestSolution.path.length() - 2)
                : bestSolution.path;
            writeOutput(completePath, expandedCount, bestSolution.g, System.nanoTime() - startTime);
        } else {
            writeOutput("No solution found.", expandedCount, Integer.MAX_VALUE, System.nanoTime() - startTime);
        }
    }
    

    ////////////////////DFID////////////////////

    private void dfid() {
        long startTime = System.nanoTime();
        createdStates = 0; // Reset counter
        cost = 0; // Reset cost
        for (int depth = 1; ; depth++) {
            Set<Board> visited = new HashSet<>();
            String result = limitedDFS(initialBoard, depth, visited, "", 0); // Match the method signature
            if (!result.equals("cutoff")) {
                long endTime = System.nanoTime();
                double timeSeconds = (endTime - startTime) / 1_000_000_000.0;
                writeOutput(result, createdStates, cost, endTime - startTime);
                return;
            }
        }
    }
    
    
    private String limitedDFS(Board current, int depth, Set<Board> visited, String path, int currentCost) {
        Stack<Node> stack = new Stack<>();
        stack.push(new Node(current, currentCost, path, 0));

        if (withOpenList) {
            System.out.println("Open list:");
            for (Node node : stack) {
                System.out.println(node.state); // 
            }
        }

        if (current.equals(goalBoard)) {
            cost = currentCost; // Update the total cost when the goal is reached
            return path.substring(0, path.length() - 2); 
        }
        if (depth == 0) {
            return "cutoff";
        }
    
        visited.add(current);
        boolean cutoffOccurred = false;
    
        for (Action action : current.getPossibleActions()) {
            Board next = current.applyAction(action);
            createdStates++; 
    
            if (!visited.contains(next)) { // Check if the state has already been visited
                String result = limitedDFS(next, depth - 1, visited,
                        path + action.toString() + "--", currentCost + action.cost);
                if (result.equals("cutoff")) {
                    cutoffOccurred = true;
                } else if (!result.equals("fail")) {
                    return result; // Return as soon as a valid path is found
                }
            }
        }
    
        visited.remove(current); // Remove the state when backtracking
        return cutoffOccurred ? "cutoff" : "fail";
    }
    
    

    private int heuristic(Board state) {
        int h = 0;
        String[][] current = state.getBoard();
        String[][] goal = goalBoard.getBoard();
    
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                String marble = current[i][j];
                if (!marble.equals("_") && !marble.equals("X")) {
                    for (int x = 0; x < 3; x++) {
                        for (int y = 0; y < 3; y++) {
                            if (goal[x][y].equals(marble)) {
                                int distance = Math.abs(i - x) + Math.abs(j - y);
                                int tileCost = Board.getCostMap().get(marble); 
                                h += distance * tileCost; // Weight distance by the tile cost
                            }
                        }
                    }
                }
            }
        }
        return h;
    }


    private void printOpenList(Collection<Node> open) {
        System.out.println("Open list:");
        for (Node node : open) System.out.println(node.state);
    }
    


    private void writeOutput(String path, int numExpanded, int cost, long timeNano) {
        try (PrintWriter writer = new PrintWriter("output.txt")) {
            writer.println(path.equals("no path") ? "no path" : path.endsWith("--") ? path.substring(0, path.length() - 2) : path);
            writer.println("Num: " + numExpanded);
            writer.println("Cost: " + (cost == Integer.MAX_VALUE ? "inf" : cost));
            if (withTime) writer.printf("%.3f seconds%n", timeNano / 1_000_000_000.0);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    

    private static class Node {
        Board state;
        int g; // Cost so far
        int f; // Total cost (f = g + h)
        String path;
    
        Node(Board state, int g, String path, int f) {
            this.state = state;
            this.g = g;
            this.f = f;
            this.path = path;
        }
    
        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            Node node = (Node) o;
            return state.equals(node.state);
        }
    
        @Override
        public int hashCode() {
            return Objects.hash(state);
        }
    }
    
}