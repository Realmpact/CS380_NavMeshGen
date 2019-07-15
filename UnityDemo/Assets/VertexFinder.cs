using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class NavMeshVertex
{
    public NavMeshVertex()
    {
        id = -1;
        neighbours = new List<NavMeshVertex>();
        cost = new List<float>();
        normal = Vector3.zero;
        Reset();
    }

    public void Reset()
    {
        totalCost = Mathf.Infinity;
        parent = null;
        isVisited = false;
    }

    public int id;
    public Vector3 point;
    public List<NavMeshVertex> neighbours;
    public List<float> cost;
    public float totalCost;
    public NavMeshVertex parent;
    public bool isVisited;
    public Vector3 normal;
}

public class VertexFinder : MonoBehaviour
{
    List<NavMeshVertex> vertices;

    // pre calculate path
    // find neighbour
    // profit

    void Start()
    {
        vertices = new List<NavMeshVertex>();

        NavMeshTriangulation nmt = NavMesh.CalculateTriangulation();
        foreach (Vector3 vec in nmt.vertices)
        {
            bool isAdd = true;
            foreach (var vtx in vertices)
                if (vtx.point == vec)
                {
                    isAdd = false;
                    break;
                }

            if (isAdd)
            {
                NavMeshVertex toAdd = new NavMeshVertex();
                toAdd.point = vec;
                vertices.Add(toAdd);
            }
        }

        Debug.Log("Vertex count: " + vertices.Count);

        foreach (NavMeshVertex vtx in vertices)
        {
            //vtx.neighbours = new List<NavMeshVertex>();
            foreach (NavMeshVertex neighbour in vertices)
            {
                if (vtx.point == neighbour.point)
                    continue;
               
                Vector3 dir = (neighbour.point - vtx.point);
                float dist = Vector3.Distance(vtx.point, neighbour.point);
                if (!Physics.Raycast(vtx.point, dir.normalized, dist))
                {
                    vtx.neighbours.Add(neighbour);
                    vtx.cost.Add(dist);
                }
            }
        }
    }

    int testIndexOne = 0;
    int testIndexTwo = 1;
    void Update()
    {
        /*
        if (Application.isPlaying)
        {
            if (testIndexOne < vertices.Count - 1 && Input.GetKeyDown(KeyCode.A))
                ++testIndexOne;
            else if (testIndexOne >= 0 && Input.GetKeyDown(KeyCode.S))
                --testIndexOne;
            
            if (testIndexTwo < vertices.Count && Input.GetKeyDown(KeyCode.K))
                ++testIndexTwo;
            else if (testIndexTwo >= 1 && Input.GetKeyDown(KeyCode.L))
                --testIndexTwo;
        }
        */
    }


    private void OnDrawGizmos()
    {
        if (vertices == null || vertices.Count == 0)
            return;

        Gizmos.color = Color.red;
        List<Vector3> path = new List<Vector3>();
        if (GetPath(vertices[testIndexOne], vertices[vertices.Count - testIndexTwo], out path))
        {
            Debug.Log("path found");
            for (int i = 0; i < path.Count - 1; ++i)
            {
                Gizmos.DrawLine(path[i], path[i + 1]);
                Gizmos.DrawSphere(path[i + 1], 0.1f);
            }
        }
    }

    public bool GetPath(NavMeshVertex origin, NavMeshVertex goal, out List<Vector3> path)
    {
        path = new List<Vector3>();

        List<NavMeshVertex> openList = new List<NavMeshVertex>();

        origin.totalCost = 0.0f;
        openList.Add(origin);

        while (openList.Count > 0)
        {
            // Find cheapest node
            int cheapestIndex = 0;
            for (int i = 1; i < openList.Count; ++i)
            {
                if (openList[i].totalCost < openList[cheapestIndex].totalCost)
                    cheapestIndex = i;
            }
            var currPoint = openList[cheapestIndex];
            openList.RemoveAt(cheapestIndex);

            // Reached
            if (currPoint == goal)
            {
                while (currPoint != origin)
                {
                    path.Insert(0, currPoint.point);
                    currPoint = currPoint.parent;
                }
                path.Insert(0, origin.point);
                Debug.Log(path.Count);

                foreach (var vtx in vertices)
                    vtx.Reset();

                return true;
            }

            // Loop neighbours
            for (int i = 0; i < currPoint.neighbours.Count; ++i)
            {
                // Skip visited vertex
                if (currPoint.neighbours[i].isVisited)
                    continue;

                float newCost = currPoint.totalCost + currPoint.cost[i];
                if (newCost < currPoint.neighbours[i].totalCost)
                {
                    currPoint.neighbours[i].totalCost = newCost;
                    currPoint.neighbours[i].parent = currPoint;
                }

                if (!openList.Contains(currPoint.neighbours[i]))
                    openList.Add(currPoint.neighbours[i]);
            }

            currPoint.isVisited = true;
        }

        foreach (var vtx in vertices)
            vtx.Reset();

        return false;
    }
}
