using System.Collections.Generic;
using UnityEngine;

public class pathfinding : MonoBehaviour
{
    public Transform target;             // The target transform to reach
    public LayerMask obstacleLayer;      // Layer to detect obstacles
    public float maxDistance = 10f;      // Maximum distance to search for a path
    public GameObject ship;
    GameObject obstructionObject;
    private Vector3 boxCenter;
    private Vector3 boxSize;
    private MeshRenderer[] objectColliders;
    private Vector3? obstructionPoint;   // Position where the box cast hits an obstacle
    GameObject lastHitObject;
    public List<Vector3> waypoints = new List<Vector3>();
    public List<GameObject> visited = new List<GameObject>();
    Vector3 lastDirection;
    void Start()
    {   visited.Clear();
        CalculateBoundingBox(ship);
        FindPath(transform.position, target.position);
    }

Vector3 startPoint;
void FindPath(Vector3 start, Vector3 end)
{
    waypoints.Clear();
    visited.Clear();
    obstructionPoint = null; // Reset the obstruction point
    // Check if the path is clear directly to the target
    if (IsPathClear(start, end, boxCenter, boxSize, obstacleLayer)) 
    {
        waypoints.Add(target.position);
        Debug.Log("Way Free");
        return; // Exit early if the path is clear
    }
    lastHitObject = obstructionObject;
    
    Debug.Log("Obstruction detected, searching for alternative path...");

    CheckAllDirections(start, end);
}


void CheckAllDirections(Vector3 start, Vector3 end, int depth = 0)
{   
    // Set a maximum recursion depth to prevent infinite loops
    int maxDepth = 50;
    if (depth > maxDepth)
    {
        Debug.Log("Max recursion depth reached, exiting to avoid infinite loop.");
        return;
    }

    startPoint = start;
    float stepSize = boxSize.x;
    Vector3[] directions = { Vector3.up, Vector3.down, Vector3.left, Vector3.right };
    float[] searchDistances = new float[directions.Length];
    
    // Limit for forward movement to prevent infinite loop
    float maxForwardDistance = 100.0f;
    float forwardDistance = 0;

    // Move forward as long as the path is clear and within the max forward limit
    while (IsStepClear(startPoint + Vector3.forward * stepSize, boxSize * 2, obstacleLayer) && forwardDistance < maxForwardDistance)
    {
        startPoint += Vector3.forward * stepSize;

        #warning minimize waypoints... remove and fix
        waypoints.Add(startPoint); 

        forwardDistance += stepSize;
        Debug.Log("Moving forward, new start point: " + startPoint);
    }

    // Check directions from the updated startPoint
    while (true)
    {
        bool anyDirectionValid = false;

        for (int i = 0; i < directions.Length; i++)
        {   
            Vector3 direction = directions[i];
            Vector3 newStart = startPoint + direction * searchDistances[i];

            // Ensure the step is within bounds and clear
            if (searchDistances[i] <= maxDistance && IsStepClear(newStart, boxSize, obstacleLayer))
            {
                // If path to end is clear, finalize path
                if (IsPathClear(newStart, end, boxCenter, boxSize, obstacleLayer))
                {
                    if(waypoints.Count == 1)waypoints.RemoveAt(0);
                    waypoints.Add(newStart);
                    waypoints.Add(end);
                    Debug.Log($"Path found towards {direction}: {newStart}");
                    return;
                }

                // Handle obstacles and recursion with depth control
                else if (lastHitObject != obstructionObject && !visited.Contains(obstructionObject))
                {
                    Debug.Log("New Object Hit " + obstructionObject.name);
                    visited.Add(lastHitObject);
                    lastHitObject = obstructionObject;

                    if (IsOppositeDirection(direction, lastDirection))
                    {
                        if (waypoints.Count != 0)
                        {
                            waypoints.RemoveAt(waypoints.Count - 1);
                        }
                    }

                    waypoints.Add(newStart);
                    lastDirection = direction;
                    CheckAllDirections(newStart, end, depth + 1);  // Recursive call with depth tracking
                    return;
                }

                // Increment search distance and mark direction as valid for further search
                searchDistances[i] += stepSize;
                anyDirectionValid = true;
            }
            else
            {
                Debug.Log($"Path blocked in {direction} at distance {searchDistances[i]}");
            }
        }

        if (!anyDirectionValid)
        {
            Debug.Log("No clear path found in any direction.");
            break;
        }
    }
}


// Check if a specific step location is clear of obstacles
bool IsStepClear(Vector3 position, Vector3 boxSize, LayerMask obstacleLayer)
{
    // Perform an overlap box to check if the step position is within an obstacle
    Collider[] hitColliders = Physics.OverlapBox(position, boxSize / 2, Quaternion.identity, obstacleLayer);
    return hitColliders.Length == 0;
}

// Method to check if two directions are opposite
private bool IsOppositeDirection(Vector3 lastDirection, Vector3 currentDirection)
{
    return (lastDirection == Vector3.up && currentDirection == Vector3.down) ||
           (lastDirection == Vector3.down && currentDirection == Vector3.up) ||
           (lastDirection == Vector3.left && currentDirection == Vector3.right) ||
           (lastDirection == Vector3.right && currentDirection == Vector3.left);
}



bool IsPathClear(Vector3 start, Vector3 end, Vector3 boxCenter, Vector3 boxSize, LayerMask obstacleLayer)
{
    Vector3 direction = (end - start).normalized;
    float distance = Vector3.Distance(start, end);

    // Perform the box cast and capture hit info
    if (Physics.BoxCast(start + boxCenter, boxSize / 2, direction, out RaycastHit hitInfo, Quaternion.identity, distance, obstacleLayer))
    {
        obstructionObject = hitInfo.collider.gameObject;
        
        if (Vector3.Distance(start, hitInfo.point) <= distance)
        {
            return false; // Obstruction found, path not clear
        }
    }

    return true; // Path is clear
}












    private void CalculateBoundingBox(GameObject ship)
    {
        if (ship == null) return;

        objectColliders = ship.GetComponentsInChildren<MeshRenderer>();

        if (objectColliders.Length == 0)
        {
            Debug.LogError("No colliders found on the object to spawn.");
            return;
        }

        Bounds bounds = objectColliders[0].bounds;

        foreach (MeshRenderer col in objectColliders)
        {
            bounds.Encapsulate(col.bounds);
        }

        boxCenter = bounds.center - ship.transform.position;
        boxSize = bounds.size;
    }



private void OnDrawGizmos()
{   
    waypoints.Clear();
    visited.Clear();
    CalculateBoundingBox(ship);
    FindPath(transform.position, target.position);
    // Draw rays from each waypoint to the target
    Gizmos.color = Color.green; // Color for the rays
        for (int i = 0; i < waypoints.Count - 1; i++)
        {
            Debug.DrawLine(waypoints[i], waypoints[i + 1], Color.green);
        }Debug.DrawLine(transform.position, waypoints[0]); // Draw a line from waypoint to target
    foreach (var waypoint in waypoints)
    {
        Gizmos.DrawSphere(waypoint, 0.2f);
    }




    /*Gizmos.color = Color.cyan; // Color for the path boxes
    for (int i = 0; i < waypoints.Count - 1; i++)
    {
        Vector3 start = waypoints[i];
        Vector3 end = waypoints[i + 1];

        // Calculate the midpoint for the box position
        Vector3 midPoint = (start + end) / 2;

        // Draw the box along the path, centered at the midpoint
        Gizmos.DrawWireCube(midPoint, boxSize); // Draw a wire cube at the midpoint with the ship's bounding box size
    }*/




    /*
    Vector3 direction = (target.position - transform.position).normalized; // Direction towards the target
    float distance = Vector3.Distance(transform.position, target.position);
    Gizmos.color = Color.red;
    Gizmos.DrawRay(transform.position, direction * distance); // Draw the ray towards the target


    // Draw the box cast at the distance where the box cast hit if there is an obstruction
    if (obstructionPoint.HasValue)
    {
        float obstructionDistance = Vector3.Distance(transform.position, obstructionPoint.Value); // Calculate the distance to the obstruction
        Vector3 drawPosition = transform.position + direction * obstructionDistance; // Position to draw the cube along the ray

        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(drawPosition, boxSize); // Draw the cube at the calculated position along the ray
    }
    */
}

}
// add boxed path
// use jobs