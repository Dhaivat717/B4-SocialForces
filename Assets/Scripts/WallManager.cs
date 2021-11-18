using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WallManager : MonoBehaviour
{
    public static WallManager inst;
    public GameObject prefabW;
    private List<Agent> walls = new List<Agent>();
    public float wallProb = 0.3f;
    public float wsp = 20;
    private GameObject wp;
    private static HashSet<GameObject> allWalls = new HashSet<GameObject>();


    void Start()
    {
        inst = this;
        Random.InitState(0);

        wp = GameObject.Find("Walls");
        for (int i = -Mathf.RoundToInt(wsp / 2); i < wsp / 2; i++)
            for (int j = -Mathf.RoundToInt(wsp / 2); j < wsp / 2; j++)
            {
                if (Random.value < wallProb)
                {
                    GameObject wall = null;
                    wall = Instantiate(prefabW, new Vector3(i + 0.5f, 0.5f, j + 0.5f), Quaternion.identity);
                    wall.name = "Wall " + i;
                    wall.transform.parent = wp.transform;
                    var wallScript = wall.GetComponent<Agent>();

                    walls.Add(wallScript);
                    allWalls.Add(wall);
                }
            }

    }
    #region Public Functions

    public static bool IsWall(GameObject obj)
    {
        return allWalls.Contains(obj);
    }

    #endregion
}
