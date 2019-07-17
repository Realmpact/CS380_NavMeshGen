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
    public bool isKey;

    public Vertex(int i, Vector3 pos, Vector3 nrm)
    {
        id = i;
        position = pos;
        normal = nrm;
        neighbours = new Dictionary<int, Vertex>();
        isKey = false;
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
    public Vector3 normal;
    public List<NavmeshTriangle> neighbours;

    public NavmeshTriangle(NavMeshVertex a, NavMeshVertex b, NavMeshVertex c)
    {
        v0 = a;
        v1 = b;
        v2 = c;
        normal = Vector3.zero;
        neighbours = new List<NavmeshTriangle>();
    }

    public bool IsConnected(NavMeshVertex v)
    {
        return v0.id == v.id || v1.id == v.id || v2.id == v.id;
    }

    public bool IsAdjacent(NavmeshTriangle triangle)
    {
        int commonEdge = 0;
        if (v0.id == triangle.v0.id || v0.id == triangle.v1.id || v0.id == triangle.v2.id) ++commonEdge;
        if (v1.id == triangle.v0.id || v1.id == triangle.v1.id || v1.id == triangle.v2.id) ++commonEdge;
        if (v2.id == triangle.v0.id || v2.id == triangle.v1.id || v2.id == triangle.v2.id) ++commonEdge;
        return commonEdge == 2;
    }

    public void AddNeighbour(NavmeshTriangle triangle)
    {
        neighbours.Add(triangle);

        if (triangle.IsConnected(v0))
        {
            if (v0.id != triangle.v0.id)
                if (!v0.neighbours.Contains(triangle.v0)) v0.neighbours.Add(triangle.v0);
            if (v0.id != triangle.v1.id)
                if (!v0.neighbours.Contains(triangle.v1)) v0.neighbours.Add(triangle.v1);
            if (v0.id != triangle.v2.id)
                if (!v0.neighbours.Contains(triangle.v2)) v0.neighbours.Add(triangle.v2);
        }
        if (triangle.IsConnected(v1))
        {
            if (v1.id != triangle.v0.id)
                if (!v1.neighbours.Contains(triangle.v0)) v1.neighbours.Add(triangle.v0);
            if (v1.id != triangle.v1.id)
                if (!v1.neighbours.Contains(triangle.v1)) v1.neighbours.Add(triangle.v1);
            if (v1.id != triangle.v2.id)
                if (!v1.neighbours.Contains(triangle.v2)) v1.neighbours.Add(triangle.v2);
        }
        if (triangle.IsConnected(v2))
        {
            if (v2.id != triangle.v0.id)
                if (!v2.neighbours.Contains(triangle.v0)) v2.neighbours.Add(triangle.v0);
            if (v2.id != triangle.v1.id)
                if (!v2.neighbours.Contains(triangle.v1)) v2.neighbours.Add(triangle.v1);
            if (v2.id != triangle.v2.id)
                if (!v2.neighbours.Contains(triangle.v2)) v2.neighbours.Add(triangle.v2);
        }
    }
}

public class NavmeshGenerator : MonoBehaviour
{
    public Material lineMat;     // material of line used to draw
    public Material fillMat;
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
    public Dictionary<int, NavMeshVertex> uniqueNavmeshVertices = new Dictionary<int, NavMeshVertex>();

    int index = 0;

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.L))
        {
            ComputeNavmesh();
        }

        if (Input.GetKeyDown(KeyCode.K))
        {
            index = (index + 1) % uniqueVertices.Count;
        }

        /*int i = 0;
        foreach (KeyValuePair<int, Vertex> vertexPair in uniqueVertices)
        {
            if (i == index)
            {
                Debug.DrawRay(vertexPair.Value.position, vertexPair.Value.normal, Color.white);
                foreach (KeyValuePair<int, Vertex> neighbourPair in vertexPair.Value.neighbours)
                {
                    Debug.DrawRay(neighbourPair.Value.position, neighbourPair.Value.normal, Color.red);
                }
            }
            i++;
        }*/
    }

    void MyDrawLine(Vector3 start, Vector3 end, Material lineMat)
    {
        GL.Begin(GL.LINES); // start draw
        lineMat.SetPass(0);
        // set color
        GL.Color(new Color(lineMat.color.r, lineMat.color.g, lineMat.color.b, lineMat.color.a));
        // start and end points
        GL.Vertex3(start.x, start.y, start.z);
        GL.Vertex3(end.x, end.y, end.z);
        GL.End(); // end draw
    }

    void FillTriangle(Vector3 v0, Vector3 v1, Vector3 v2, Material lineMat)
    {
        GL.PushMatrix();
        lineMat.SetPass(0);
        GL.Begin(GL.TRIANGLES);
        // set color
        GL.Color(new Color(lineMat.color.r, lineMat.color.g, lineMat.color.b, lineMat.color.a));
        GL.Vertex3(v0.x, v0.y, v0.z);
        GL.Vertex3(v1.x, v1.y, v1.z);
        GL.Vertex3(v2.x, v2.y, v2.z);
        GL.End();
        GL.PopMatrix();
    }

    void DrawNavMesh()
    {
        foreach (NavmeshTriangle triangle in allNavMeshTriangles)
        {
            Vector3[] vtx = new Vector3 [3];
            vtx[0] = triangle.v0.point;
            vtx[1] = triangle.v1.point;
            vtx[2] = triangle.v2.point;
            
            MyDrawLine(vtx[0], vtx[1], lineMat);
            MyDrawLine(vtx[1], vtx[2], lineMat);
            MyDrawLine(vtx[2], vtx[0], lineMat);
            
            FillTriangle(vtx[0], vtx[1], vtx[2], fillMat);
        }
    }

    void OnPostRender()
    {
        DrawNavMesh();
    }

    public void ClearNavmesh()
    {
        allTriangles.Clear();
        allPolygons.Clear();
        uniqueVertices.Clear();
        uniqueNavmeshVertices.Clear();
        allNavMeshTriangles.Clear();
    }

    public void ComputeNavmesh()
    {
        if (!meshFilter) return;

        GameObject[] obstacles = GameObject.FindGameObjectsWithTag("NavMeshObstacle");

        Mesh mesh = meshFilter.mesh;
        Matrix4x4 matrix = meshFilter.transform.localToWorldMatrix;
        ClearNavmesh();

        Debug.Log("Begin");

        float cosSlope = Mathf.Cos(slopeAngle * Mathf.Deg2Rad);
        for (int i = 0; i < mesh.triangles.Length; i += 3)
        {
            int idxTri0 = mesh.triangles[i];
            int idxTri1 = mesh.triangles[i + 1];
            int idxTri2 = mesh.triangles[i + 2];

            Vertex v0 = new Vertex(idxTri0, matrix.MultiplyPoint3x4(mesh.vertices[idxTri0]), matrix.MultiplyVector(mesh.normals[idxTri0]));
            Vertex v1 = new Vertex(idxTri1, matrix.MultiplyPoint3x4(mesh.vertices[idxTri1]), matrix.MultiplyVector(mesh.normals[idxTri1]));
            Vertex v2 = new Vertex(idxTri2, matrix.MultiplyPoint3x4(mesh.vertices[idxTri2]), matrix.MultiplyVector(mesh.normals[idxTri2]));

            //Merge vertices that are too close
            foreach (KeyValuePair<int, Vertex> vertexPair in uniqueVertices)
            {
                if (Vector3.Distance(v0.position, vertexPair.Value.position) <= 0.25f)
                {
                    idxTri0 = vertexPair.Key;
                    v0 = vertexPair.Value;
                }
                else if (Vector3.Distance(v1.position, vertexPair.Value.position) <= 0.25f)
                {
                    idxTri1 = vertexPair.Key;
                    v1 = vertexPair.Value;
                }
                else if (Vector3.Distance(v2.position, vertexPair.Value.position) <= 0.25f)
                {
                    idxTri2 = vertexPair.Key;
                    v2 = vertexPair.Value;
                }
            }

            v0.position += v0.normal.normalized * meshHeight;
            v1.position += v1.normal.normalized * meshHeight;
            v2.position += v2.normal.normalized * meshHeight;

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

        Debug.Log("Identify Key Points");

        List<int> keyIds = new List<int>();
        foreach (KeyValuePair<int, Vertex> pair in uniqueVertices)
        {
            Vertex vertex = pair.Value;
            List<int> invalidId = new List<int>();
            foreach (KeyValuePair<int, Vertex> neighbourPair in vertex.neighbours)
            {
                Vertex neighbour = neighbourPair.Value;
                Ray ray = new Ray(vertex.position, (neighbour.position - vertex.position).normalized);
                float dist = (neighbour.position - vertex.position).magnitude;
                RaycastHit hitInfo;
                foreach (GameObject obstacle in obstacles)
                {
                    Collider collider = obstacle.GetComponent<Collider>();
                    if (!collider) continue;

                    if (collider.ClosestPoint(neighbour.position) == neighbour.position)
                    {
                        invalidId.Add(neighbour.id);
                        keyIds.Add(pair.Key);
                    }
                   else if (collider.Raycast(ray, out hitInfo, dist))
                    {
                        invalidId.Add(neighbour.id);
                        keyIds.Add(pair.Key);
                        keyIds.Add(neighbourPair.Key);
                    }
                }
            }
            foreach (int id in invalidId)
            {
                vertex.neighbours.Remove(id);
            }
        }

        foreach (int id in keyIds)
        {
            Vertex vert = uniqueVertices[id];
            vert.isKey = true;
            uniqueVertices[id] = vert;
        }

        Debug.Log("Group Vertices");
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

        Debug.Log("Group Polygons");
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

        Debug.Log("Remove redundant vertices");
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
            if (vertex.Value.neighbours.Count > 3 * multipier && !vertex.Value.isKey)
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

        //Merge close vertices
        /*foreach (KeyValuePair<int, Vertex> vertex1 in uniqueVertices)
        {
            foreach (KeyValuePair<int, Vertex> vertex2 in uniqueVertices)
            {
                if (vertex1.Key == vertex2.Key) continue;
            }
        }*/

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
                }
            }
        }

        Debug.Log("Gen Navmesh Triangles");
        
        for (int i = allPolygons.Count - 1; i >= 0; --i)
        {
            List<Vertex> temp = allPolygons[i].vertices;
            Triangulate(temp, ref obstacles);
        }

        foreach (KeyValuePair<int, NavMeshVertex> navVertexPair in uniqueNavmeshVertices)
        {
            Vertex correspondance = uniqueVertices[navVertexPair.Value.id];
            foreach (KeyValuePair<int, Vertex> vertexPair in correspondance.neighbours)
            {
                navVertexPair.Value.neighbours.Add(uniqueNavmeshVertices[vertexPair.Value.id]);
            }
        }

        for (int i = 0; i < allNavMeshTriangles.Count; ++i)
        {
            for (int j = i + 1; j < allNavMeshTriangles.Count; ++j)
            {
                if (allNavMeshTriangles[i].IsAdjacent(allNavMeshTriangles[j]))
                { 
                    allNavMeshTriangles[i].AddNeighbour(allNavMeshTriangles[j]);
                    allNavMeshTriangles[j].AddNeighbour(allNavMeshTriangles[i]);
                }
            }
        }
    }

    void CheckCollisions(ref List<Vertex> vertices, ref GameObject[] obstacles)
    {
        for (int i = 0; i < vertices.Count; ++i)
        {
            Vector3 startPos = vertices[i].position;
            for (int j = i + 1; j < vertices.Count; ++j)
            {
                Vector3 direction = (vertices[j].position - startPos).normalized;
                Ray ray = new Ray();
                ray.origin = startPos;
                ray.direction = direction;

                foreach (GameObject obstacle in obstacles)
                {
                    Collider collider = obstacle.GetComponent<Collider>();
                }
            }
        }
    }

    void Triangulate(List<Vertex> vertices, ref GameObject[] obstacles)
    { 
        vertices.Sort((a, b) => a.position.x.CompareTo(b.position.x));

        for (int i = 0; i < vertices.Count; ++i)
        {
            for (int j = i + 1; j < vertices.Count; ++j)
            {
                for (int k = j + 1; (k % vertices.Count) != i; ++k)
                {
                    Vertex v0 = vertices[i];
                    Vertex v1 = vertices[j % vertices.Count];
                    Vertex v2 = vertices[k % vertices.Count];
                    int idxTri0 = v0.id;
                    int idxTri1 = v1.id;
                    int idxTri2 = v2.id;

                    Ray ray0 = new Ray(v0.position, (v1.position - v0.position).normalized);
                    Ray ray1 = new Ray(v1.position, (v2.position - v1.position).normalized);
                    Ray ray2 = new Ray(v2.position, (v0.position - v2.position).normalized);
                    float dist0 = (v1.position - v0.position).magnitude;
                    float dist1 = (v2.position - v1.position).magnitude;
                    float dist2 = (v0.position - v2.position).magnitude;
                    RaycastHit hitInfo;

                    bool skip = false;
                    foreach (GameObject obstacle in obstacles)
                    {
                        Collider collider = obstacle.GetComponent<Collider>();
                        if (!collider) continue;

                        if (collider.Raycast(ray0, out hitInfo, dist0) ||
                        collider.Raycast(ray1, out hitInfo, dist1) ||
                        collider.Raycast(ray2, out hitInfo, dist2))
                        {
                            skip = true;
                            break;
                        }
                        else
                        {
                            Vector3 p0 = collider.ClosestPoint(v0.position);
                            Vector3 p1 = collider.ClosestPoint(v1.position);
                            Vector3 p2 = collider.ClosestPoint(v2.position);

                            if (IsPointInTriangle(v0.position, v1.position, v2.position, p0) ||
                                IsPointInTriangle(v0.position, v1.position, v2.position, p1) ||
                                IsPointInTriangle(v0.position, v1.position, v2.position, p2))
                            {
                                skip = true;
                                break;
                            }
                        }
                    }
                    if (skip) continue;

                    if (!uniqueVertices[idxTri0].neighbours.ContainsKey(idxTri1)) uniqueVertices[idxTri0].neighbours.Add(idxTri1, v1);
                    if (!uniqueVertices[idxTri0].neighbours.ContainsKey(idxTri2)) uniqueVertices[idxTri0].neighbours.Add(idxTri2, v2);
                    if (!uniqueVertices[idxTri1].neighbours.ContainsKey(idxTri0)) uniqueVertices[idxTri1].neighbours.Add(idxTri0, v0);
                    if (!uniqueVertices[idxTri1].neighbours.ContainsKey(idxTri2)) uniqueVertices[idxTri1].neighbours.Add(idxTri2, v2);
                    if (!uniqueVertices[idxTri2].neighbours.ContainsKey(idxTri1)) uniqueVertices[idxTri2].neighbours.Add(idxTri1, v1);
                    if (!uniqueVertices[idxTri2].neighbours.ContainsKey(idxTri0)) uniqueVertices[idxTri2].neighbours.Add(idxTri0, v0);

                    NavMeshVertex nv0, nv1, nv2;
                    if (!uniqueNavmeshVertices.ContainsKey(idxTri0))
                    {
                        nv0 = new NavMeshVertex();
                        nv0.id = v0.id;
                        nv0.point = v0.position;
                        nv0.normal = v0.normal;
                        uniqueNavmeshVertices.Add(idxTri0, nv0);
                    }
                    else nv0 = uniqueNavmeshVertices[idxTri0];
                    if (!uniqueNavmeshVertices.ContainsKey(idxTri1))
                    {
                        nv1 = new NavMeshVertex();
                        nv1.id = v1.id;
                        nv1.point = v1.position;
                        nv1.normal = v1.normal;
                        uniqueNavmeshVertices.Add(idxTri1, nv1);
                    }
                    else nv1 = uniqueNavmeshVertices[idxTri1];
                    if (!uniqueNavmeshVertices.ContainsKey(idxTri2))
                    {
                        nv2 = new NavMeshVertex();
                        nv2.id = v2.id;
                        nv2.point = v2.position;
                        nv2.normal = v2.normal;
                        uniqueNavmeshVertices.Add(idxTri2, nv2);
                    }
                    else nv2 = uniqueNavmeshVertices[idxTri2];

                    NavmeshTriangle triangle = new NavmeshTriangle(nv0, nv1, nv2);
                    allNavMeshTriangles.Add(triangle);
                }
            }
        }
    }

    public bool IsPointInTriangle(Vector3 v0, Vector3 v1, Vector3 v2, Vector3 pt)
    {
        Vector3 test0 = (v0 - pt).normalized;
        Vector3 test1 = (v1 - pt).normalized;
        Vector3 test2 = (v2 - pt).normalized;
        float angle = 0.0f;

        angle += Vector3.Angle(test0, test1);
        angle += Vector3.Angle(test1, test2);
        angle += Vector3.Angle(test2, test0);

        float floatingCheck = Mathf.Abs(360.0f - angle);

        return floatingCheck <= 1.0f;
    }
}
