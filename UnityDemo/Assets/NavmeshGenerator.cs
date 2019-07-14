using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public enum NormalType
{
    NONE,
    POSITIVE_Y,
    NEGATIVE_Y,
    POSITIVE_X,
    NEGATIVE_X,
    POSITVE_Z,
    NEGATIVE_Z,
    CUSTOM_AXIS
}

public struct Vertex
{
    public int id;
    public Vector3 position;
    public Vector3 normal;
    public Dictionary<int, Vertex> neighbours;

    public Vertex(int i, Vector3 pos, Vector3 nrm)
    {
        id = i;
        position = pos;
        normal = nrm;
        neighbours = new Dictionary<int, Vertex>();
    }

    public bool IsSame(Vertex v)
    {
        return id == v.id; 
    }
}

public struct Triangle
{
    public Vertex v0;
    public Vertex v1;
    public Vertex v2;
    public Vector3 normal;

    public Triangle(Vertex a, Vertex b, Vertex c)
    {
        v0 = a;
        v1 = b;
        v2 = c;
        normal = ((v0.normal + v1.normal + v2.normal) / 3).normalized;
    }

    public bool IsAdjacent(Triangle triangle)
    {
        return (v0.IsSame(triangle.v0) || v0.IsSame(triangle.v1) || v0.IsSame(triangle.v2) ||
            v1.IsSame(triangle.v0) || v1.IsSame(triangle.v1) || v1.IsSame(triangle.v2) ||
            v2.IsSame(triangle.v0) || v2.IsSame(triangle.v1) || v2.IsSame(triangle.v2));
    }

    public bool IsConnected(Vertex v)
    {
        return v0.IsSame(v) || v1.IsSame(v) || v2.IsSame(v);
    }
}

public struct Polygon
{
    public List<Vertex> vertices;
    public Vector3 normal;

    public Polygon(Triangle triangle)
    {
        vertices = new List<Vertex>();
        vertices.Add(triangle.v0);
        vertices.Add(triangle.v1);
        vertices.Add(triangle.v2);
        normal = triangle.normal;
    }

    public bool IsConnected(Vertex v)
    {
        for (int i = 0; i < vertices.Count; ++i)
        {
            if (vertices[i].IsSame(v))
            {
                return true;
            }
        }

        return false;
    }

    public bool IsConnected(Triangle triangle)
    {
        for (int i = 0; i < vertices.Count; ++i)
        {
            if (triangle.IsConnected(vertices[i]))
            {
                return true;
            }
        }

        return false;
    }

    public bool IsConnected(Polygon polygon)
    {
        for (int i = 0; i < vertices.Count; ++i)
        {
            if (polygon.IsConnected(vertices[i]))
            {
                return true;
            }
        }

        return false;
    }

    public void AddTriangle(Triangle triangle)
    {
        if (!IsConnected(triangle.v0)) vertices.Add(triangle.v0);
        if (!IsConnected(triangle.v1)) vertices.Add(triangle.v1);
        if (!IsConnected(triangle.v2)) vertices.Add(triangle.v2);
    }

    public void AddPolygon(Polygon polygon)
    {
        for (int i = 0; i < polygon.vertices.Count; ++i)
        {
            if (!IsConnected(polygon.vertices[i])) vertices.Add(polygon.vertices[i]);
        }
    }

    public void RemoveUnnecessaryVertices(int ignore, ref List<Polygon> polygons)
    {
        for (int i = vertices.Count - 1; i >= 0; --i)
        {
            bool toRemove = true;
            for (int j = 0; j < polygons.Count; ++j)
            {
                if (j == ignore) continue;

                if (polygons[j].IsConnected(vertices[i]))
                {
                    toRemove = false;
                    break;
                }
            }

            if (toRemove) vertices.RemoveAt(i);
        }
    }
}

public struct NavmeshTriangle
{
    public NavMeshVertex v0;
    public NavMeshVertex v1;
    public NavMeshVertex v2;
    public List<NavmeshTriangle> neighbours;

    public NavmeshTriangle(Vertex a, Vertex b, Vertex c)
    {
        v0 = new NavMeshVertex();
        v0.point = a.position;
        v0.id = a.id;
        v1 = new NavMeshVertex();
        v1.point = b.position;
        v1.id = b.id;
        v2 = new NavMeshVertex();
        v2.point = c.position;
        v2.id = c.id;
        neighbours = new List<NavmeshTriangle>();
    }

    public bool CheckNeighbour(NavmeshTriangle triangle)
    {
        return (v0.id == triangle.v0.id || v0.id == triangle.v1.id || v0.id == triangle.v2.id
            || v1.id == triangle.v0.id || v1.id == triangle.v1.id || v1.id == triangle.v2.id
            || v2.id == triangle.v0.id || v2.id == triangle.v1.id || v2.id == triangle.v2.id);
    }
}

public class NavmeshGenerator : MonoBehaviour
{
    public MeshFilter meshFilter;

    public NormalType typeNormal = NormalType.NONE;
    //0-90
    public float slopeAngle = 0f;
    public float meshGap = 0f;
    public float meshHeight = 0f;

    public List<Triangle> allTriangles = new List<Triangle>();
    public List<Polygon> allPolygons = new List<Polygon>();
    public List<NavmeshTriangle> allNavMeshTriangles = new List<NavmeshTriangle>();
    public Dictionary<int, Vertex> uniqueVertices = new Dictionary<int, Vertex>();

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.L))
        {
            ComputeNavmesh();
        }

        foreach (Polygon polygon in allPolygons)
        {
            foreach (Vertex vertex in polygon.vertices)
            {
                Debug.DrawRay(vertex.position, vertex.normal, Color.black);

                foreach (KeyValuePair<int, Vertex> neighbour in uniqueVertices[vertex.id].neighbours)
                {
                    Debug.DrawLine(vertex.position, neighbour.Value.position, Color.red);
                }
            }
        }
    }

    public void ComputeNavmesh()
    {
        if (!meshFilter) return;

        Mesh mesh = meshFilter.mesh;
        allTriangles.Clear();
        allPolygons.Clear();
        uniqueVertices.Clear();

        float cosSlope = Mathf.Cos(slopeAngle * Mathf.Deg2Rad);
        for (int i = 0; i < mesh.triangles.Length; i += 3)
        {
            int idxTri0 = mesh.triangles[i];
            int idxTri1 = mesh.triangles[i + 1];
            int idxTri2 = mesh.triangles[i + 2];

            Vertex v0 = new Vertex(idxTri0, mesh.vertices[idxTri0], mesh.normals[idxTri0]);
            Vertex v1 = new Vertex(idxTri1, mesh.vertices[idxTri1], mesh.normals[idxTri1]);
            Vertex v2 = new Vertex(idxTri2, mesh.vertices[idxTri2], mesh.normals[idxTri2]);
            Triangle triangle = new Triangle(v0, v1, v2);
            if (Vector3.Dot(Vector3.up, triangle.normal) >= cosSlope)
            {
                allTriangles.Add(new Triangle(v0, v1, v2));

                if (!uniqueVertices.ContainsKey(idxTri0)) uniqueVertices.Add(idxTri0, v0);
                if (!uniqueVertices.ContainsKey(idxTri1)) uniqueVertices.Add(idxTri1, v1);
                if (!uniqueVertices.ContainsKey(idxTri2)) uniqueVertices.Add(idxTri2, v2);

                if (!uniqueVertices[idxTri0].neighbours.ContainsKey(idxTri1)) uniqueVertices[idxTri0].neighbours.Add(idxTri1, v1);
                if (!uniqueVertices[idxTri0].neighbours.ContainsKey(idxTri2)) uniqueVertices[idxTri0].neighbours.Add(idxTri2, v2);
                if (!uniqueVertices[idxTri1].neighbours.ContainsKey(idxTri0)) uniqueVertices[idxTri1].neighbours.Add(idxTri0, v0);
                if (!uniqueVertices[idxTri1].neighbours.ContainsKey(idxTri2)) uniqueVertices[idxTri1].neighbours.Add(idxTri2, v2);
                if (!uniqueVertices[idxTri2].neighbours.ContainsKey(idxTri1)) uniqueVertices[idxTri2].neighbours.Add(idxTri1, v1);
                if (!uniqueVertices[idxTri2].neighbours.ContainsKey(idxTri0)) uniqueVertices[idxTri2].neighbours.Add(idxTri0, v0);
            }
        }

        for (int i = 0; i < allTriangles.Count; ++i)
        {
            Triangle triangle = allTriangles[i];
            Polygon polygon = new Polygon(triangle);
            for (int j = allTriangles.Count - 1; j > i; --j)
            {
                Triangle neighbour = allTriangles[j];
                if (Vector3.Dot(polygon.normal, neighbour.normal) == 1.0f)
                {
                    if (polygon.IsConnected(neighbour))
                    {
                        polygon.AddTriangle(neighbour);
                        allTriangles.RemoveAt(j);
                    }
                }

            }
            allPolygons.Add(polygon);
        }

        bool change = false;
        do
        {
            change = false;
            for (int i = 0; i < allPolygons.Count; ++i)
            {
                Polygon polygon = allPolygons[i];
                for (int j = allPolygons.Count - 1; j > i; --j)
                {
                    Polygon neighbour = allPolygons[j];
                    if (Vector3.Dot(polygon.normal, neighbour.normal) == 1.0f)
                    {
                        if (polygon.IsConnected(neighbour))
                        {
                            polygon.AddPolygon(neighbour);
                            allPolygons.RemoveAt(j);
                            change = true;
                        }
                    }
                }
            }
        }
        while (change);

        List<int> idsRemove = new List<int>();
        foreach (KeyValuePair<int, Vertex> vertex in uniqueVertices)
        {
            int multipier = 0;
            foreach (Polygon polygon in allPolygons)
            {
                if (polygon.IsConnected(vertex.Value))
                {
                    ++multipier;
                }
            }
            if (vertex.Value.neighbours.Count > 3 * multipier)
            {
                idsRemove.Add(vertex.Key);
            }
        }
        foreach (KeyValuePair<int, Vertex> vertex in uniqueVertices)
        {
            if (idsRemove.Contains(vertex.Key)) continue;

            foreach (int id in idsRemove)
            {
                if (vertex.Value.neighbours.ContainsKey(id)) vertex.Value.neighbours.Remove(id);
            }
        }
        foreach (int id in idsRemove)
        {
            uniqueVertices.Remove(id);
        }

        for (int i = allPolygons.Count - 1; i >= 0; --i)
        {
            for (int j = allPolygons[i].vertices.Count - 1; j >= 0; --j)
            {
                if (!uniqueVertices.ContainsKey(allPolygons[i].vertices[j].id))
                {
                    allPolygons[i].vertices.RemoveAt(j);
                }
            }

            if (allPolygons[i].vertices.Count < 3) allPolygons.RemoveAt(i);
            else
            {
                for (int j = 0; j < allPolygons[i].vertices.Count - 1; ++j)
                {
                    int id = allPolygons[i].vertices[j].id;
                    uniqueVertices[id].neighbours.Clear();
                    /*for (int k = j + 1; k < allPolygons[i].vertices.Count; ++k)
                    {
                        int neighbourId = allPolygons[i].vertices[k].id;
                        if (!uniqueVertices[id].neighbours.ContainsKey(neighbourId)) uniqueVertices[id].neighbours.Add(neighbourId, allPolygons[i].vertices[k]);
                        if (!uniqueVertices[neighbourId].neighbours.ContainsKey(id)) uniqueVertices[neighbourId].neighbours.Add(id, allPolygons[i].vertices[j]);
                    }*/
                }
            }
        }

        allNavMeshTriangles.Clear();
        for (int i = allPolygons.Count - 1; i >= 0; --i)
        {
            Triangulate(allPolygons[i].vertices);
        }

        for (int i = 0; i < allNavMeshTriangles.Count; ++i)
        {
            for (int j = i + 1; j < allNavMeshTriangles.Count; ++j)
            {
                if (allNavMeshTriangles[i].CheckNeighbour(allNavMeshTriangles[j]))
                {
                    allNavMeshTriangles[i].neighbours.Add(allNavMeshTriangles[j]);
                    allNavMeshTriangles[j].neighbours.Add(allNavMeshTriangles[i]);
                }
            }
        }
    }

    void Triangulate(List<Vertex> vertices)
    { 
        vertices.Sort((a, b) => a.position.x.CompareTo(b.position.x));

        for (int i = 0; i < vertices.Count - 2; ++i)
        {
            Vertex v0 = vertices[i];
            Vertex v1 = vertices[i + 1];
            Vertex v2 = vertices[i + 2];
            int idxTri0 = v0.id;
            int idxTri1 = v1.id;
            int idxTri2 = v2.id;

            if (!uniqueVertices[idxTri0].neighbours.ContainsKey(idxTri1)) uniqueVertices[idxTri0].neighbours.Add(idxTri1, v1);
            if (!uniqueVertices[idxTri0].neighbours.ContainsKey(idxTri2)) uniqueVertices[idxTri0].neighbours.Add(idxTri2, v2);
            if (!uniqueVertices[idxTri1].neighbours.ContainsKey(idxTri0)) uniqueVertices[idxTri1].neighbours.Add(idxTri0, v0);
            if (!uniqueVertices[idxTri1].neighbours.ContainsKey(idxTri2)) uniqueVertices[idxTri1].neighbours.Add(idxTri2, v2);
            if (!uniqueVertices[idxTri2].neighbours.ContainsKey(idxTri1)) uniqueVertices[idxTri2].neighbours.Add(idxTri1, v1);
            if (!uniqueVertices[idxTri2].neighbours.ContainsKey(idxTri0)) uniqueVertices[idxTri2].neighbours.Add(idxTri0, v0);

            NavmeshTriangle triangle = new NavmeshTriangle(v0, v1, v2);
            allNavMeshTriangles.Add(triangle);
        }
    }
}
