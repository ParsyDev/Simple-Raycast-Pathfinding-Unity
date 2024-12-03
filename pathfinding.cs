using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Pathfinding : MonoBehaviour
{   
    #if UNITY_EDITOR
    public Transform target;             // The target transform to reach
    public GameObject shipp;
    #endif
    public float maxDistance = 1000f;      // Maximum distance to search for a path
    public float bufferMultiplier = 0.3;
    GameObject obstructionObject;
    private Vector3 boxCenter;
    private Vector3 boxSize;
    private MeshRenderer[] objectColliders;
    private Vector3? obstructionPoint;   // Position where the box cast hits an obstacle
    GameObject lastHitObject;
    public List<Vector3> waypoints = new List<Vector3>();
    public List<Vector3> waypointList = new List<Vector3>();
    public List<GameObject> visited = new List<GameObject>();
    Vector3 lastDirection;
    Vector3 lastHitPoint;

    Vector3 startPoint;
    public List<Vector3> FindPath(Vector3 start, Vector3 end , GameObject ship)
    {   
        CalculateBoundingBox(end, ship);
        waypointList.Clear();
        waypoints.Clear();
        visited.Clear();
        obstructionPoint = null; // Reset the obstruction point

        // Check if the path is clear directly to the target
        if (IsPathClear(start, end, boxCenter, boxSize * bufferMultiplier, ship))
        {
            waypoints.Add(end);
            Debug.Log("Way Free");
            return waypoints; // Return early if the path is clear
        }

        lastHitObject = obstructionObject;
        Debug.Log("Obstruction detected, searching for alternative path...");
        CheckAllDirections(start, end, ship);
        
        return waypointList; // Return the waypoint list after finding the path
    }


void CheckAllDirections(Vector3 start, Vector3 end, GameObject ship, int depth = 0)
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
    Vector3[] directions = { ship.transform.up, -ship.transform.up, ship.transform.right, -ship.transform.right, ship.transform.forward};
    float[] searchDistances = new float[directions.Length];
    
    
    // Limit for forward movement to prevent infinite loop
    float forwardDistance = 0;
    Vector3 forwardDir = (end - startPoint).normalized;
    // Move forward as long as the path is clear and within the max forward limit
    while (IsStepClear(startPoint + forwardDir * stepSize, boxSize * bufferMultiplier, ship) && forwardDistance < maxDistance)
    {
        startPoint += forwardDir * stepSize;

        waypoints.Add(startPoint); 
        if (IsPathClear(startPoint, end, boxCenter, boxSize * bufferMultiplier, ship))
        {
            if(waypoints.Count == 1)waypoints.RemoveAt(0);
            waypointList.Add(startPoint);
            waypointList.Add(end);
            waypoints.Add(startPoint);
            waypoints.Add(end);
            return;
        }
        forwardDistance += stepSize;
        Debug.Log("Moving forward, new start point: " + startPoint);

    }

    #warning later modify to go back because this is not the solution
    startPoint -= forwardDir * stepSize; 


    while (true)
    {
        bool anyDirectionValid = false;
        for (int i = 0; i < directions.Length; i++)
        {   
            Vector3 direction = directions[i];
            Vector3 newStart = startPoint + direction * searchDistances[i];
            
            // Ensure the step is within bounds and clear
            if (searchDistances[i] <= maxDistance && IsStepClear(newStart, boxSize * bufferMultiplier, ship))
            {
                // If path to end is clear, finalize path
                if (IsPathClear(newStart, end, boxCenter, boxSize * bufferMultiplier, ship))
                {
                    if(waypoints.Count == 1)waypoints.RemoveAt(0);
                    waypointList.Add(newStart);
                    waypointList.Add(end);
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
                    waypointList.Add(newStart);
                    if (IsOppositeDirection(direction, lastDirection))
                    {
                        if (waypoints.Count != 0)
                        {
                            waypoints.RemoveAt(waypoints.Count - 1);
                        }
                    }

                    waypoints.Add(newStart);
                    
                    lastDirection = direction;
                    CheckAllDirections(newStart, end, ship, depth + 1);  // Recursive call with depth tracking
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


bool IsStepClear(Vector3 position, Vector3 boxSize, GameObject parent)
{
    Collider[] hitColliders = Physics.OverlapBox(position, boxSize / 2, Quaternion.identity);

    foreach (Collider collider in hitColliders)
    {
        if (IsChildOf(collider.gameObject, parent))
        {
            continue; // Ignore the parent or its child objects
        }
        return false; // Found an obstruction
    }
    return true; // No obstructions
}





bool IsPathClear(Vector3 start, Vector3 end, Vector3 boxCenter, Vector3 boxSize, GameObject parent)
{
    Vector3 direction = (end - start).normalized;
    float distance = Vector3.Distance(start, end);
    float boxSizeMagnitude = boxSize.magnitude;

    RaycastHit[] hits = Physics.BoxCastAll(start + boxCenter, boxSize / 2, direction, Quaternion.identity, distance);

    foreach (RaycastHit hit in hits)
    {
        GameObject obstructionObject = hit.collider.gameObject;

        lastHitPoint = hit.point;
        if (IsChildOf(obstructionObject, parent))
        {
            continue; // Ignore this hit and keep checking others
        }
        else
        {
            return false;
        }
        // Check if the hit point is within the obstructing threshold
        if (Vector3.Distance(start, hit.point) < distance - boxSizeMagnitude)
        {
            return false; // Obstruction found, path not clear
        }
    }

    return true; // No valid obstruction found, path is clear
}


// Helper function to check if a GameObject is a child of the specified parent
bool IsChildOf(GameObject obj, GameObject parent)
{
    Transform current = obj.transform.root;
    Transform current2 = parent.transform.root;
    while (current != null)
    {
        if (current == current2)
        {
            return true; // Object is the parent or a child of the parent
        }
        current = current.parent;
    }
    return false;
}

// Method to check if two directions are opposite
private bool IsOppositeDirection(Vector3 lastDirection, Vector3 currentDirection)
{
    return (lastDirection == Vector3.up && currentDirection == Vector3.down) ||
           (lastDirection == Vector3.down && currentDirection == Vector3.up) ||
           (lastDirection == Vector3.left && currentDirection == Vector3.right) ||
           (lastDirection == Vector3.right && currentDirection == Vector3.left);
}








    private void CalculateBoundingBox(Vector3 end ,GameObject ship)
    {
        if (ship == null) return;
        // Get all colliders in the prefab's children
        objectColliders = ship.GetComponentsInChildren<MeshRenderer>();

        if (objectColliders.Length == 0)
        {
            Debug.LogError("No colliders found on the object to spawn.");
            return;
        }

        // Initialize bounds based on the first collider
        Bounds bounds = objectColliders[0].bounds;

        // Expand bounds to include all other colliders
        foreach (MeshRenderer col in objectColliders)
        {
            bounds.Encapsulate(col.bounds);
        }

        // Calculate center and size for the bounding box
        boxCenter = bounds.center - ship.transform.position;
        boxSize = bounds.size;
    }

    


#if UNITY_EDITOR
void OnDrawGizmosSelected() 
{
    waypoints.Clear();
    visited.Clear();
    CalculateBoundingBox(target.position, shipp);
    FindPath(shipp.transform.position, target.position, shipp);
}

private void OnDrawGizmos()
{   
    Gizmos.color = Color.green; // Color for the boxes

        // Draw 3D boxes between waypoints
        for (int i = 0; i < waypoints.Count - 1; i++)
        {
            DrawBoxBetweenPoints(waypoints[i], waypoints[i + 1]);
        }

        // Draw a line from the transform position to the first waypoint
        if (waypoints.Count > 0)
        {
            DrawBoxBetweenPoints(transform.position, waypoints[0]);
        }

        // Draw spheres at each waypoint for visualization
        foreach (var waypoint in waypoints)
        {
            Gizmos.DrawSphere(waypoint, 0.2f);
        }
        foreach (var waypoint in waypointList)
        {
            Gizmos.DrawSphere(waypoint, 1);
        }
        for (int i = 0; i < waypointList.Count - 1; i++)
        {
            DrawBoxBetweenPointss(waypointList[i], waypointList[i + 1]);
        }

        // Draw a line from the transform position to the first waypoint
        if (waypointList.Count > 0)
        {
            DrawBoxBetweenPointss(transform.position, waypointList[0]);
        }
    }

    private void DrawBoxBetweenPoints(Vector3 start, Vector3 end)
    {
        Gizmos.color = Color.green; // Color for the boxes

        Vector3 center = (start + end) / 2; // Center of the box
        float distance = Vector3.Distance(start, end); // Length of the segment

        // Calculate the rotation to align the box with the segment
        Quaternion rotation = Quaternion.LookRotation(end - start);

        // Draw the wire cube at the calculated position with rotation
        Gizmos.matrix = Matrix4x4.TRS(center, rotation, Vector3.one); // Set the Gizmos matrix for rotation and position
        Gizmos.DrawWireCube(Vector3.zero, new Vector3(boxSize.x, boxSize.y, distance)); // Draw at the origin
        Gizmos.matrix = Matrix4x4.identity; // Reset the Gizmos matrix
    }
    private void DrawBoxBetweenPointss(Vector3 start, Vector3 end)
    {
        Gizmos.color = Color.blue; // Color for the boxes

        Vector3 center = (start + end) / 2; // Center of the box
        float distance = Vector3.Distance(start, end); // Length of the segment

        // Calculate the rotation to align the box with the segment
        Quaternion rotation = Quaternion.LookRotation(end - start);

        // Draw the wire cube at the calculated position with rotation
        Gizmos.matrix = Matrix4x4.TRS(center, rotation, Vector3.one); // Set the Gizmos matrix for rotation and position
        Gizmos.DrawWireCube(Vector3.zero, new Vector3(boxSize.x + bufferMultiplier, boxSize.y + bufferMultiplier, distance)); // Draw at the origin
        Gizmos.matrix = Matrix4x4.identity; // Reset the Gizmos matrix
    }
#endif
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
// add boxed path
// use jobs