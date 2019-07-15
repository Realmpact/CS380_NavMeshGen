using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectChanger : MonoBehaviour
{
    public KeyCode hideInputKey = KeyCode.M;
    public GameObject canvas;
    public int currIndex = 0;
    public GameObject[] models;

    NavmeshGenerator nmg;

    void Start()
    {
        nmg = FindObjectOfType<NavmeshGenerator>();

        foreach (GameObject go in models)
            go.SetActive(false);

        models[currIndex].SetActive(true);
        nmg.meshFilter = models[currIndex].GetComponent<MeshFilter>();
        if (nmg.meshFilter == null)
            nmg.meshFilter = models[currIndex].GetComponentInChildren<MeshFilter>();

        Debug.Assert(nmg.meshFilter != null, "No mesh filter found");
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(hideInputKey))
            canvas.SetActive(!canvas.activeSelf);
    }

    public void SwitchModel(int dir)
    {
        ChangeModel(currIndex + dir);
    }

    void ChangeModel(int targetIndex)
    {
        models[currIndex].SetActive(false);

        if (targetIndex <= -1)
            currIndex = models.Length - 1;
        else if (targetIndex >= models.Length)
            currIndex = 0;
        else
            currIndex = targetIndex;

        models[currIndex].SetActive(true);
        nmg.meshFilter = models[currIndex].GetComponent<MeshFilter>();
        if (nmg.meshFilter == null)
            nmg.meshFilter = models[currIndex].GetComponentInChildren<MeshFilter>();

        Debug.Assert(nmg.meshFilter != null, "No mesh filter found");
    }
}
