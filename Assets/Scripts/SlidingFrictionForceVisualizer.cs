using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SlidingFrictionForceVisualizer : MonoBehaviour
{
    public GameObject player1;
    public GameObject player2;

    [Range(0, 10)]
    public float velo1;
    [Range(0, 10)]
    public float velo2;

    void Update()
    {
        Debug.DrawLine(player1.transform.position, player1.transform.position + Vector3.up * 3, Color.green);
        Debug.DrawLine(player1.transform.position, player1.transform.position + player1.transform.forward * velo1, Color.cyan);
        Debug.DrawLine(player2.transform.position, player2.transform.position + player2.transform.forward * velo2, Color.cyan);

        var n = (player2.transform.position - player1.transform.position).normalized;
        var tangent = Vector3.Cross(Vector3.up, n);
        Debug.DrawLine(player1.transform.position, player1.transform.position + tangent * 2, Color.yellow);

        var magnitude = Vector3.Dot(player1.transform.forward * velo1 - player2.transform.forward * velo2, tangent);
        Debug.DrawLine(player1.transform.position, player1.transform.position + tangent * magnitude * 2, Color.red);

        Debug.DrawLine(player1.transform.position + player1.transform.forward * velo1, player2.transform.position + player2.transform.forward * velo2, Color.magenta);

        #region Visualization

#if UNITY_EDITOR
        if (Application.isFocused)
        {
            UnityEditor.SceneView.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));
        }
#endif

        #endregion
    }
}
