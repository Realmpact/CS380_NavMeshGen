using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Robot : MonoBehaviour
{
    NavmeshGenerator nmg;
    Coroutine moveCoroutine = null;
    public bool isShowVertices = false;

    void Start()
    {
        nmg = FindObjectOfType<NavmeshGenerator>();
    }

    void Update()
    {
        MoveRobot();
    }

    Ray ray;
    RaycastHit hit;
    void MoveRobot()
    {
        ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        if (Input.GetMouseButtonDown(0))
        {
            if (Physics.Raycast(ray, out hit, 10000.0f, ~(1 << Physics.IgnoreRaycastLayer)))
            {
                List<NavMeshVertex> path;
                if (FindPath(hit.point, out path))
                {
                    if (moveCoroutine != null)
                        StopCoroutine(moveCoroutine);
                    moveCoroutine = StartCoroutine(MoveState(path));
                    Debug.Log("moving");
                }
                else
                {
                    Debug.Log("no path found");
                }
            }
        }
    }

    private void OnDrawGizmos()
    {
        if (isShowVertices)
            foreach (NavmeshTriangle tri in nmg.allNavMeshTriangles)
            {
                Gizmos.DrawSphere(tri.v0.point, 0.1f);
                Gizmos.DrawSphere(tri.v1.point, 0.1f);
                Gizmos.DrawSphere(tri.v2.point, 0.1f);
            }

        if (hit.collider == null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(Camera.main.transform.position, ray.direction * 10000.0f);
        }
        else
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(Camera.main.transform.position, hit.point);
            Gizmos.DrawSphere(hit.point, 0.1f);
        }

        /*if (nmg != null && nmg.allNavMeshTriangles != null)
        {
            foreach (NavmeshTriangle tri in nmg.allNavMeshTriangles)
            {
                if (tri.v0.isVisited)
                {
                    Gizmos.color = Color.blue;
                    foreach (NavMeshVertex vert in tri.v0.neighbours)
                    {
                        if (!vert.isVisited) Gizmos.DrawSphere(vert.point, 0.1f);
                    }
                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(tri.v0.point, 0.1f);
                }
                if (tri.v1.isVisited)
                {
                    Gizmos.color = Color.blue;
                    foreach (NavMeshVertex vert in tri.v1.neighbours)
                    {
                        if (!vert.isVisited) Gizmos.DrawSphere(vert.point, 0.1f);
                    }
                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(tri.v1.point, 0.1f);
                }
                if (tri.v2.isVisited)
                {
                    Gizmos.color = Color.blue;
                    foreach (NavMeshVertex vert in tri.v2.neighbours)
                    {
                        if (!vert.isVisited) Gizmos.DrawSphere(vert.point, 0.1f);
                    }
                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(tri.v2.point, 0.1f);
                }
            }
        }*/
    }

    IEnumerator MoveState(List<NavMeshVertex> path)
    {
        while (path.Count > 0)
        {
            transform.position = Vector3.MoveTowards(transform.position, path[0].point, 4.2f * Time.deltaTime);
            //transform.rotation = Quaternion.LookRotation(path[0].normal);

            if (Vector3.Distance(transform.position, path[0].point) <= 0.02f)
                path.RemoveAt(0);

            yield return null;
        }

        Debug.Log("reached");
    }

    public bool FindPath(Vector3 goal, out List<NavMeshVertex> outPath)
    {
        outPath = new List<NavMeshVertex>();
        foreach (NavmeshTriangle tri in nmg.allNavMeshTriangles)
        {
            tri.v0.Reset();
            tri.v1.Reset();
            tri.v2.Reset();
        }

         // Get origin and goal triangle
        int startIndex = GetTriangleIndex(transform.position);
        int goalIndex = GetTriangleIndex(goal);

        if (startIndex == -1)
        {
            Debug.Log("start out of bound");
            return false;
        }
        else if (goalIndex == -1)
        {
            Debug.Log("goal out of bound");
            return false;
        }

        NavmeshTriangle originTri = nmg.allNavMeshTriangles[startIndex];
        NavmeshTriangle goalTri = nmg.allNavMeshTriangles[goalIndex];

        // If origin and goal share the same triangle
        NavMeshVertex v1 = new NavMeshVertex();
        NavMeshVertex v2 = new NavMeshVertex();
        v1.point = transform.position;
        v2.point = goal;
        outPath.Add(v1);
        outPath.Add(v2);

        //if (originTri.v0 == goalTri.v0 && originTri.v1 == goalTri.v1 && originTri.v2 == goalTri.v2)
        if (startIndex == goalIndex)
        {
            Debug.Log("same tri");
            return true;
        }

        // Calculate origin triangle
        NavMeshVertex originTempVertex = new NavMeshVertex();
        originTempVertex.point = transform.position;
        originTempVertex.parent = null;

        originTri.v0.totalCost = Vector3.SqrMagnitude(originTri.v0.point - transform.position);
        originTri.v1.totalCost = Vector3.SqrMagnitude(originTri.v1.point - transform.position);
        originTri.v2.totalCost = Vector3.SqrMagnitude(originTri.v2.point - transform.position);

        originTri.v0.parent = originTempVertex;
        originTri.v1.parent = originTempVertex;
        originTri.v2.parent = originTempVertex;

        // Open list
        List<NavMeshVertex> openList = new List<NavMeshVertex>();
        openList.Add(originTri.v0);
        openList.Add(originTri.v1);
        openList.Add(originTri.v2);

        // Master list for reset
        //List<NavMeshVertex> dumpster = new List<NavMeshVertex>();
        //dumpster.AddRange(openList);

        while (openList.Count > 0)
        {
            // Find cheapest node
            int cheapestIndex = 0;
            for (int i = 1; i < openList.Count; ++i)
            {
                if (openList[i].totalCost < openList[cheapestIndex].totalCost)
                    cheapestIndex = i;
            }
            NavMeshVertex currPoint = openList[cheapestIndex];
            openList.RemoveAt(cheapestIndex);

            // Check vertex against goal triangle
            if (goalTri.v0 == currPoint || goalTri.v1 == currPoint || goalTri.v2 == currPoint)
            {
                while (currPoint.parent != null)
                {
                    outPath.Insert(1, currPoint);
                    currPoint = currPoint.parent;
                }

                //foreach (NavMeshVertex vtx in dumpster)
                //    vtx.Reset();

                return true;
            }
            
            // Loop neighbours
            for (int i = 0; i < currPoint.neighbours.Count; ++i)
            {
                // Skip visited vertex
                if (currPoint.neighbours[i].isVisited)
                    continue;

                float newCost = currPoint.totalCost + Vector3.SqrMagnitude(currPoint.neighbours[i].point - currPoint.point);
                if (newCost < currPoint.neighbours[i].totalCost)
                {
                    currPoint.neighbours[i].totalCost = newCost;
                    currPoint.neighbours[i].parent = currPoint;
                }

                if (!openList.Contains(currPoint.neighbours[i]))
                {
                    openList.Add(currPoint.neighbours[i]);
                    //dumpster.Add(currPoint);
                }
            }

            currPoint.isVisited = true;
        }

        //foreach (NavMeshVertex vtx in dumpster)
        //    vtx.Reset();

        return false;
    }

    public int GetTriangleIndex(Vector3 point)
    {
        for (int i = 0; i < nmg.allNavMeshTriangles.Count; ++i)
        {
            NavmeshTriangle tri = nmg.allNavMeshTriangles[i];
            Vector3[] vecs = new Vector3[] { tri.v0.point, tri.v1.point, tri.v2.point };

                // find barycenter
            Vector3 midPoint = (tri.v0.point + tri.v1.point + tri.v2.point) / 3;
            if (CheckPointInTriangle(midPoint, vecs, 1.0f)) // clockwise
            {
                if (CheckPointInTriangle(point, vecs, 1.0f))
                {
                    return i;
                }
            }
            else // anti clockwise
            {
                if (CheckPointInTriangle(point, vecs, -1.0f))
                {
                    return i;
                }
            }

            /*
            bool clockWise = false;

            // find barycenter
            Vector3 midPoint = (tri.v0.point + tri.v1.point + tri.v2.point) / 3;

            // check for outward normal
            Vector3 inward = midPoint - tri.v0.point;
            Vector3 edge = tri.v1.point - tri.v0.point;
            Vector2 edgeNormal = new Vector2(edge.z, -edge.x);
            Vector2 inward2D = new Vector2(inward.x, inward.z);

            if (Vector2.Dot(inward2D.normalized, edgeNormal.normalized) > 0.0f)
            {
                clockWise = true;
            }
                

            Vector3 diff = tri.v0.point - point;
            Vector3 line = tri.v1.point - tri.v0.point;

            Vector2 dir = new Vector2(diff.x, diff.z);
            Vector2 norm = new Vector2(line.z, -line.x);

            if (Vector2.Dot(dir.normalized, norm.normalized) < 0.0f)
                continue;

            diff = tri.v1.point - point;
            line = tri.v2.point - tri.v1.point;

            dir = new Vector2(diff.x, diff.z);
            norm = new Vector2(line.z, -line.x);

            if (Vector2.Dot(dir.normalized, norm.normalized) < 0.0f)
                continue;

            diff = tri.v2.point - point;
            line = tri.v0.point - tri.v2.point;

            dir = new Vector2(diff.x, diff.z);
            norm = new Vector2(line.z, -line.x);

            if (Vector2.Dot(dir.normalized, norm.normalized) < 0.0f)
                continue;

            return i;
            */
        }

        return -1; // Out of bound for all triangles
    }

    bool CheckPointInTriangle(Vector3 p, Vector3[] vec, float normDir)
    {
        for (int i = 0; i < 3; ++i)
        {
            int next = i == 2 ? 0 : i + 1;

            Vector3 diff = p - vec[i];
            Vector3 line = vec[next] - vec[i];

            Vector2 dir = new Vector2(diff.x, diff.z);
            Vector2 norm = new Vector2(line.z * normDir, -line.x * normDir);

            if (Vector2.Dot(dir.normalized, norm.normalized) < 0.0f)
                return false;
        }
        return true;
    }

    public NavMeshVertex GetClosestVertex(int index, Vector3 pos)
    {
        NavMeshVertex retVertex = null;
        float shortestDist = Mathf.Infinity;

        if (index < 0)
        {
            // Check through all triangles
            foreach (var tri in nmg.allNavMeshTriangles)
            {
                float d1 = Vector3.Distance(tri.v0.point, pos);
                float d2 = Vector3.Distance(tri.v1.point, pos);
                float d3 = Vector3.Distance(tri.v2.point, pos);

                if (d1 < shortestDist)
                {
                    shortestDist = d1;
                    retVertex = tri.v0;
                }

                if (d2 < shortestDist)
                {
                    shortestDist = d2;
                    retVertex = tri.v1;
                }

                if (d3 < shortestDist)
                {
                    shortestDist = d3;
                    retVertex = tri.v2;
                }
            }
        }
        else
        {
            NavmeshTriangle tri = nmg.allNavMeshTriangles[index];

            float d1 = Vector3.Distance(tri.v0.point, pos);
            float d2 = Vector3.Distance(tri.v1.point, pos);
            float d3 = Vector3.Distance(tri.v2.point, pos);

            shortestDist = d1;
            retVertex = tri.v0;

            if (d2 < shortestDist)
            {
                shortestDist = d2;
                retVertex = tri.v1;
            }

            if (d3 < shortestDist)
            {
                shortestDist = d3;
                retVertex = tri.v2;
            }
        }
        
        return retVertex;
    }
}
