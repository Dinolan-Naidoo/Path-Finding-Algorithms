using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using Priority_Queue;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class Agent : MonoBehaviour
{
    NodeNetworkCreator nodeNetwork;

    //Variables for the UI Text for the algorithm times 
    public Text dfsText;
    public Text bfsText;
    public Text dijkstraText;
    public Text AStarManText;
    public Text AStarEuText;


    //Variables for the UI Text for the percentage of nodes searched
    public Text bfsPercentage;
    public Text dfsPercentage;
    public Text dijkstraPercentage;
    public Text AStarManPercentage;
    public Text AStarEuPercentage;


    IDictionary<Vector3, Vector3> nodeParents = new Dictionary<Vector3, Vector3>();
    IDictionary<Vector3, Sprite> prevSprite = new Dictionary<Vector3, Sprite>();

    public IDictionary<Vector3, bool> walkablePositions;
    public IDictionary<Vector3, string> obstacles;

    IList<Vector3> path;

    bool solutionFound;
    string prevAlgorithm;


    bool moveCube = false;
    int j;

    // Use this for initialization
    void Start()
    {
        nodeNetwork = GameObject.Find("NodeNetwork").GetComponent<NodeNetworkCreator>();
        obstacles = GameObject.Find("NodeNetwork").GetComponent<NodeNetworkCreator>().obstacles;
        walkablePositions = nodeNetwork.walkablePositions;
    }

    void Update()
    {
        //Moving the player along the last path chosen 
        if (moveCube)
        {
            float speed = 20 / Weight(path[j]);
            float step = Time.deltaTime * speed;
            transform.position = Vector3.MoveTowards(transform.position, path[j], step * Time.fixedDeltaTime *20);
            if (transform.position.Equals(path[j]) && j >= 0)
            {
                j--;
            }
            if (j < 0)
            {
                moveCube = false;
            }
        }
    }


    //Used in A-Star Euclidean algortihm 
    int EuclideanEstimate(Vector3 node, Vector3 goal)
    {
        int EuclideanDistance = (int)Mathf.Sqrt(Mathf.Pow(node.x - goal.x, 2) +
            Mathf.Pow(node.y - goal.y, 2) +
            Mathf.Pow(node.z - goal.z, 2));

        return EuclideanDistance;
    }

    //Used in A-Star Manhattan algortihm 
    int ManhattanEstimate(Vector3 node, Vector3 goal)
    {
        int ManhattanDistance = (int)(Mathf.Abs(node.x - goal.x) +
            Mathf.Abs(node.y - goal.y) +
            Mathf.Abs(node.z - goal.z));

        return ManhattanDistance;
    }


    //Calculating the cost of each path 
    int PathChosen(Vector3 node, Vector3 goal, string path)
    {
        switch (path)
        {
            case "euclidean":
                return EuclideanEstimate(node, goal);
            case "manhattan":
                return ManhattanEstimate(node, goal);
        }
        return -1;
    }




    
    /////////////////////////////////////////A Star START
    Vector3 FindShortestPathAStar(Vector3 startPosition, Vector3 goalPosition, string selectedMethod)
    {

        uint nodeVisitCount = 0;
        float timeNow = Time.realtimeSinceStartup;

        // A* tries to minimize f(x) = g(x) + h(x), where g(x) is the distance from the start to node "x" and
        // h(x) is some heuristic that must be admissible, meaning it never overestimates the cost to the next node.
        // There are formal logical proofs you can look up that determine how heuristics are and are not admissible.

        IEnumerable<Vector3> validNodes = walkablePositions
            .Where(x => x.Value == true)
            .Select(x => x.Key);

        // Represents h(x) or the score from whatever heuristic we're using
        IDictionary<Vector3, int> heuristicScore = new Dictionary<Vector3, int>();

        // Represents g(x) or the distance from start to node "x" (Same meaning as in Dijkstra's "distances")
        IDictionary<Vector3, int> distanceFromStart = new Dictionary<Vector3, int>();

        foreach (Vector3 vertex in validNodes)
        {
            heuristicScore.Add(new KeyValuePair<Vector3, int>(vertex, int.MaxValue));
            distanceFromStart.Add(new KeyValuePair<Vector3, int>(vertex, int.MaxValue));
        }

        heuristicScore[startPosition] = PathChosen(startPosition, goalPosition, selectedMethod);
        distanceFromStart[startPosition] = 0;

        // The item dequeued from a priority queue will always be the one with the lowest int value
        //    In this case we will input nodes with their calculated distances from the start g(x),
        //    so we will always take the node with the lowest distance from the queue.
        SimplePriorityQueue<Vector3, int> priorityQueue = new SimplePriorityQueue<Vector3, int>();
        priorityQueue.Enqueue(startPosition, heuristicScore[startPosition]);

        while (priorityQueue.Count > 0)
        {
            // Get the node with the least distance from the start
            Vector3 curr = priorityQueue.Dequeue();

            //Keeping track of nodes visited
            nodeVisitCount++;

            // If our current node is the goal then stop 
            if (curr == goalPosition)
            {
                if (selectedMethod == "euclidean")
                {
                    AStarEuText.text = "Time: " + (Time.realtimeSinceStartup - timeNow).ToString("0.0000");
                    AStarEuPercentage.text = string.Format("Nodes Explored: {0} ({1:F2}%)", nodeVisitCount, (nodeVisitCount / (double)walkablePositions.Count) * 100);
                }
                else if (selectedMethod == "manhattan")
                {
                    AStarManText.text = "Time: " + (Time.realtimeSinceStartup - timeNow).ToString("0.0000");
                    AStarManPercentage.text = string.Format("Nodes Explored: {0} ({1:F2}%)", nodeVisitCount, (nodeVisitCount / (double)walkablePositions.Count) * 100);
                }

                return goalPosition;
            }

            IList<Vector3> neighbors = GetWalkableNodes(curr);

            foreach (Vector3 node in neighbors)
            {
                // Get the distance so far, add it to the distance to the neighbor
                int currScore = distanceFromStart[curr] + Weight(node);

                // If our distance to this neighbor is LESS than another calculated shortest path
                //    to this neighbor, set a new node parent and update the scores as our current
                //    best for the path so far.
                if (currScore < distanceFromStart[node])
                {
                    nodeParents[node] = curr;
                    distanceFromStart[node] = currScore;

                    int hScore = distanceFromStart[node] + PathChosen(node, goalPosition, selectedMethod);
                    heuristicScore[node] = hScore;

                    // If this node isn't already in the queue, make sure to add it. Since the
                    //    algorithm is always looking for the smallest distance, any existing entry
                    //    would have a higher priority anyway.
                    if (!priorityQueue.Contains(node))
                    {
                        priorityQueue.Enqueue(node, hScore);
                    }
                }
            }
        }

        return startPosition;
    }

    /////////////////////////////////////////A Star END


    /////////////////////////////////////////DIJKSTRA START 
    //Populates IList<Vector3> path with a valid solution to the goalPosition.
    //Returns the goalPosition if a solution is found.
    //Returns the startPosition if no solution is found.
    Vector3 FindShortestPathDijkstra(Vector3 startPosition, Vector3 goalPosition)
    {

        uint nodeVisitCount = 0;
        float timeNow = Time.realtimeSinceStartup;

        //A priority queue containing the shortest distance so far from the start to a given node
        IPriorityQueue<Vector3, int> priority = new SimplePriorityQueue<Vector3, int>();

        //A list of all nodes that are walkable, initialized to have infinity distance from start
        IDictionary<Vector3, int> distances = walkablePositions
            .Where(x => x.Value == true)
            .ToDictionary(x => x.Key, x => int.MaxValue);

        //Our distance from the start to itself is 0
        distances[startPosition] = 0;
        priority.Enqueue(startPosition, 0);

        while (priority.Count > 0)
        {

            Vector3 curr = priority.Dequeue();
            nodeVisitCount++;

            if (curr == goalPosition)
            {
                // If the goal position is the lowest position in the priority queue then there are
                //    no other nodes that could possibly have a shorter path.
                print("Dijkstra: " + distances[goalPosition]);
                print("Dijkstra time: " + (Time.realtimeSinceStartup - timeNow).ToString());
                dijkstraText.text = "Time: " + (Time.realtimeSinceStartup - timeNow).ToString("0.0000");
                print(string.Format("Dijkstra visits: {0} ({1:F2}%)", nodeVisitCount, (nodeVisitCount / (double)walkablePositions.Count) * 100));
                dijkstraPercentage.text = string.Format("Nodes Explored: {0} ({1:F2}%)", nodeVisitCount, (nodeVisitCount / (double)walkablePositions.Count) * 100);
                return goalPosition;
            }

            IList<Vector3> nodes = GetWalkableNodes(curr);

            //Look at each neighbor to the node
            foreach (Vector3 node in nodes)
            {

                int dist = distances[curr] + Weight(node);

                //If the distance to the parent, PLUS the distance added by the neighbor,
                //is less than the current distance to the neighbor,
                //update the neighbor's paent to curr, update its current best distance
                if (dist < distances[node])
                {
                    distances[node] = dist;
                    nodeParents[node] = curr;

                    if (!priority.Contains(node))
                    {
                        priority.Enqueue(node, dist);
                    }
                }
            }
        }

        return startPosition;
    }

    /////////////////////////////////////////DIJKSTRA END



    /////////////////////////////////////////BREATH FIRST SEARCH START
    //Populates IList<Vector3> path with a valid solution to the goalPosition.
    //Returns the goalPosition if a solution is found.
    //Returns the startPosition if no solution is found.
    Vector3 FindShortestPathBFS(Vector3 startPosition, Vector3 goalPosition)
    {

        uint nodeVisitCount = 0;
        float timeNow = Time.realtimeSinceStartup;

        Queue<Vector3> queue = new Queue<Vector3>();
        HashSet<Vector3> exploredNodes = new HashSet<Vector3>();
        queue.Enqueue(startPosition);

        while (queue.Count != 0)
        {
            Vector3 currentNode = queue.Dequeue();
            nodeVisitCount++;

            if (currentNode == goalPosition)
            {
                print("BFS time: " + (Time.realtimeSinceStartup - timeNow).ToString());
                bfsText.text = "Time: " + (Time.realtimeSinceStartup - timeNow).ToString("0.0000");
                print(string.Format("BFS visits: {0} ({1:F2}%)", nodeVisitCount, (nodeVisitCount / (double)walkablePositions.Count) * 100));
                bfsPercentage.text = string.Format("Nodes Explored: {0} ({1:F2}%)", nodeVisitCount, (nodeVisitCount / (double)walkablePositions.Count) * 100);

                return currentNode;
            }

            IList<Vector3> nodes = GetWalkableNodes(currentNode);

            foreach (Vector3 node in nodes)
            {
                if (!exploredNodes.Contains(node))
                {
                    //Mark the node as explored
                    exploredNodes.Add(node);

                    //Store a reference to the previous node
                    nodeParents.Add(node, currentNode);

                    //Add this to the queue of nodes to examine
                    queue.Enqueue(node);
                }
            }
        }

        return startPosition;
    }

    /////////////////////////////////////////BREATH FIRST SEARCH END


    /////////////////////////////////////////DEPTH FIRST SEARCH START
    //Populates IList<Vector3> path with a valid solution to the goalPosition.
    //Returns the goalPosition if a solution is found.
    //Returns the startPosition if no solution is found.
    Vector3 FindShortestPathDFS(Vector3 startPosition, Vector3 goalPosition)
    {

        uint nodeVisitCount = 0;
        float timeNow = Time.realtimeSinceStartup;

        Stack<Vector3> stack = new Stack<Vector3>();
        HashSet<Vector3> exploredNodes = new HashSet<Vector3>();
        stack.Push(startPosition);

        while (stack.Count != 0)
        {
            Vector3 currentNode = stack.Pop();
            nodeVisitCount++;

            if (currentNode == goalPosition)
            {
                print("DFS time: " + (Time.realtimeSinceStartup - timeNow).ToString());
                dfsText.text = "Time: " + (Time.realtimeSinceStartup - timeNow).ToString("0.0000");
                print(string.Format("DFS visits: {0} ({1:F2}%)", nodeVisitCount, (nodeVisitCount / (double)walkablePositions.Count) * 100));
                dfsPercentage.text = string.Format("Nodes Explored: {0} ({1:F2}%)", nodeVisitCount, (nodeVisitCount / (double)walkablePositions.Count) * 100);
                return currentNode;
            }

            IList<Vector3> nodes = GetWalkableNodes(currentNode);

            foreach (Vector3 node in nodes)
            {
                if (!exploredNodes.Contains(node))
                {
                    //Mark the node as explored
                    exploredNodes.Add(node);

                    //Store a reference to the previous node
                    nodeParents.Add(node, currentNode);

                    //Add this to the queue of nodes to examine
                    stack.Push(node);
                }
            }
        }

        return startPosition;
    }
    /////////////////////////////////////////DEPTH FIRST SEARCH END


    //Function to check whether the player can move to a node 
    bool CanMove(Vector3 nextPosition)
    {
        return (walkablePositions.ContainsKey(nextPosition) ? walkablePositions[nextPosition] : false);
    }

    //Function to display the shortest path found for any selected algorithm
    public void DisplayShortestPath(string algorithm)
    {

        if (solutionFound && algorithm == prevAlgorithm)
        {
            foreach (Vector3 node in path)
            {
                nodeNetwork.nodeReference[node].GetComponent<SpriteRenderer>().sprite = prevSprite[node];
            }

            solutionFound = false;
            return;
        }

        nodeParents = new Dictionary<Vector3, Vector3>();
        path = FindShortestPath(algorithm);

        if (path == null)
            return;

        //Loading the relevant sprites for each path 
        Sprite BFSTile = Resources.Load<Sprite>("BFS");
        Sprite DFSTile = Resources.Load<Sprite>("DFS");
        Sprite dijkstraTile = Resources.Load<Sprite>("Dijkstra");
        Sprite AStarManTile = Resources.Load<Sprite>("AStarMan");
        Sprite AStarEuTile = Resources.Load<Sprite>("AStarEu");
        Sprite goalTile = Resources.Load<Sprite>("Goal");

        foreach (Vector3 node in path)
        {

            prevSprite[node] = nodeNetwork.nodeReference[node].GetComponent<SpriteRenderer>().sprite;

            if (algorithm == "DFS")
            {
                nodeNetwork.nodeReference[node].GetComponent<SpriteRenderer>().sprite = DFSTile;
            }
            else if (algorithm == "BFS")
            {
                nodeNetwork.nodeReference[node].GetComponent<SpriteRenderer>().sprite = BFSTile;
            }
            else if (algorithm == "AStarEuclid")
            {
                nodeNetwork.nodeReference[node].GetComponent<SpriteRenderer>().sprite = AStarEuTile;
            }
            else if (algorithm == "AStarManhattan")
            {
                nodeNetwork.nodeReference[node].GetComponent<SpriteRenderer>().sprite = AStarManTile;
            }
            else
            {
                nodeNetwork.nodeReference[node].GetComponent<SpriteRenderer>().sprite = dijkstraTile;
            }
        }

        nodeNetwork.nodeReference[path[0]].GetComponent<SpriteRenderer>().sprite = goalTile;

        j = path.Count - 1;

        solutionFound = true;
        prevAlgorithm = algorithm;
    }

    
    public void MoveCube()
    {
        moveCube = true;
    }

    IList<Vector3> FindShortestPath(string algorithm)
    {

        IList<Vector3> path = new List<Vector3>();
        Vector3 goal;
        if (algorithm == "DFS")
        {
            goal = FindShortestPathDFS(this.transform.localPosition, GameObject.Find("Goal").transform.localPosition);
        }
        else if (algorithm == "BFS")
        {
            goal = FindShortestPathBFS(this.transform.localPosition, GameObject.Find("Goal").transform.localPosition);
        }
        else if (algorithm == "AStarEuclid")
        {
            goal = FindShortestPathAStar(this.transform.localPosition, GameObject.Find("Goal").transform.localPosition, "euclidean");
        }
        else if (algorithm == "AStarManhattan")
        {
            goal = FindShortestPathAStar(this.transform.localPosition, GameObject.Find("Goal").transform.localPosition, "manhattan");
        }
        else
        {
            goal = FindShortestPathDijkstra(this.transform.localPosition, GameObject.Find("Goal").transform.localPosition);
        }

        if (goal == this.transform.localPosition || !nodeParents.ContainsKey(nodeParents[goal]))
        {
            //No solution was found.
            return null;
        }

        Vector3 curr = goal;
        while (curr != this.transform.localPosition)
        {
            path.Add(curr);
            curr = nodeParents[curr];
        }

        return path;
    }

    IList<Vector3> GetWalkableNodes(Vector3 curr)
    {

        IList<Vector3> walkableNodes = new List<Vector3>();

        IList<Vector3> possibleNodes = new List<Vector3>() {
            new Vector3 (curr.x + 1, curr.y, curr.z),
            new Vector3 (curr.x - 1, curr.y, curr.z),
            new Vector3 (curr.x, curr.y, curr.z + 1),
            new Vector3 (curr.x, curr.y, curr.z - 1),
            new Vector3 (curr.x + 1, curr.y, curr.z + 1),
            new Vector3 (curr.x + 1, curr.y, curr.z - 1),
            new Vector3 (curr.x - 1, curr.y, curr.z + 1),
            new Vector3 (curr.x - 1, curr.y, curr.z - 1)
        };

        foreach (Vector3 node in possibleNodes)
        {
            if (CanMove(node))
            {
                walkableNodes.Add(node);
            }
        }



        return walkableNodes;
    }

    int Weight(Vector3 node)
    {
        if (obstacles.Keys.Contains(node))
        {
            if (obstacles[node] == "slow")
            {
                return 3;
            }
            else if (obstacles[node] == "verySlow")
            {
                return 5;
            }
            else
            {
                return 1;
            }
        }
        else
        {
            return 1;
        }
    }
    public void ResetFunction()
    {
        SceneManager.LoadScene("SampleScene");
    }


}
