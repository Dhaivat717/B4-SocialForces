using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wall : MonoBehaviour
{

    void Awake()
    {
        if (transform.eulerAngles.y == 90)
        {
            transform.localScale = new Vector3(transform.localScale.z, transform.localScale.y, transform.localScale.x);

            transform.eulerAngles = new Vector3(0, 0, 0);
        }
    }
}
