using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class Agent : MonoBehaviour
{
    private Rigidbody rBody;
    private Vector3 wf = Vector3.zero;
    private List<Vector3> p;
    public float radius;
    public const float DESIRED_VELOCITY = 5f;
    private NavMeshAgent nam;
    private HashSet<GameObject> pn = new HashSet<GameObject>();

    private bool vp = false;
    private bool vn = true;
    private bool vw = false;

    public float perceptionRadius;
    public float mass;


    void Start()
    {
        p = new List<Vector3>();
        nam = GetComponent<NavMeshAgent>();
        rBody = GetComponent<Rigidbody>();
        float rds = radius;
        float prds = perceptionRadius;

        gameObject.transform.localScale = new Vector3(2 * rds, 1, 2 * rds);
        nam.radius = rds;
        rBody.mass = mass;
        GetComponent<SphereCollider>().radius = prds / 2;
    }

    private void Update()
    {

        if (isldr)
        {
            float vtl = Input.GetAxis("Vertical");
            float hz = Input.GetAxis("Horizontal");

            rBody.AddForce(new Vector3(hz, 0.0f, vtl) * 100);
        }
        float dis = Vector3.Distance(transform.position, p[0]);

        if (p.Count > 1 && dis < 1.1f)
        {
            p.RemoveAt(0);
        }

        else if (p.Count == 1 && dis < 2f)
        {
            p.RemoveAt(0);

            if (p.Count == 0)
            {
                pn.Clear();
                gameObject.SetActive(false);
                AgentManager.RemoveAgent(gameObject);
            }
        }

        #region Visualization

        if (vn)
        {
            foreach (var neighbor in pn)
            {
                Debug.DrawLine(transform.position, neighbor.transform.position, Color.yellow);
            }
        }

        if (vp)
        {
            if (p.Count > 0)
            {
                Debug.DrawLine(transform.position, p[0], Color.blue);
            }

            for (int i = 0; i < p.Count - 1; i++)
            {
                Debug.DrawLine(p[i], p[i + 1], Color.red);
            }
        }

        #endregion
    }

    #region Public Functions



    public Vector3 GetVelocity()
    {
        return rBody.velocity;
    }

    public void ComputePath(Vector3 destination)
    {
        nam.enabled = true;
        var nmPath = new NavMeshPath();
        nam.CalculatePath(destination, nmPath);
        p = nmPath.corners.Skip(1).ToList();
        nam.enabled = false;
    }

    #endregion

    #region Part 1



    private Vector3 CalculateAgentForce()
    {
        Vector3 fce = Vector3.zero;

        foreach (var agt in pn)
        {

            float ards = agt.gameObject.GetComponent<Agent>().radius + radius;
            float aolap = Math.Max(ards - Vector3.Distance(transform.position, agt.transform.position), 0);

            Vector3 dir = (transform.position - agt.transform.position).normalized;

            float dt = -Vector3.Distance(transform.position, agt.transform.position);

            fce += Parameters.A * Mathf.Exp(dt / Parameters.B) * dir;

            fce += aolap * Parameters.k * dt * dir;

            Vector3 tg = Vector3.Cross(Vector3.up, dir);
            fce += aolap * Parameters.Kappa * Vector3.Dot(rBody.velocity - agt.gameObject.GetComponent<Rigidbody>().velocity, tg) * tg;

        }
        return fce;
    }

    private Vector3 ComputeForce()
    {
        var f = Vector3.zero;
        f += CalculateAgentForce();

        if (hasldr)
        {
            f += CalculateLeaderForce();
        }

        wf = Vector3.zero;
        if (f != wf)
        {
            return f.normalized * Mathf.Min(f.magnitude, Parameters.maxSpeed);
        }
        else
        {
            return Vector3.zero;
        }

    }

    private Vector3 CalculateGoalForce()
    {
        Vector3 vel = GetVelocity();
        Vector3 dir = (p[0] - transform.position).normalized;

        Vector3 fce = mass * (DESIRED_VELOCITY * dir - vel);

        return fce;
    }


    public void ApplyForce()
    {
        var fce = ComputeForce();
        fce.y = 0;

        rBody.AddForce(fce * 10, ForceMode.Force);
    }

    private void CalculateWallForce(Collision collision)
    {
        wf += (-(collision.contacts[0].point - transform.position).normalized) * 1.0f * mass;
        return;
    }



    public void OnTriggerExit(Collider o)
    {
        if (pn.Contains(o.gameObject))
        {
            pn.Remove(o.gameObject);
        }
    }

    public void OnTriggerEnter(Collider o)
    {
        if (AgentManager.IsAgent(o.gameObject))
        {
            pn.Add(o.gameObject);
        }
    }

    public void OnCollisionExit(Collision collision)
    {
        return;
    }

    public void OnCollisionEnter(Collision c)
    {
        if (c.gameObject.tag == "wall")
        {
            lcwall = c.gameObject;
        }

        if (WallManager.IsWall(c.gameObject))
        {
            CalculateWallForce(c);
        }
    }

    #endregion

    #region Part 2

    #region Single-Agent Behaviors

    GameObject lcwall = null;
    float maxWallDist;


    private Vector3 CalculateSpiralForce()
    {

        Vector3 toCen = Vector3.zero - new Vector3(transform.position.x, 0, transform.position.z);
        Vector3 tg = Vector3.Cross(toCen, Vector3.up);

        Vector3 nf = tg;
        nf += toCen;

        Debug.DrawLine(transform.position, transform.position + tg, Color.blue);
        Debug.DrawLine(transform.position, transform.position + toCen, Color.red);

        return nf * mass;
    }

    private Vector3 CalculateWallFollowForce()
    {

        if (lcwall == null)
        {
            return Vector3.zero;
        }

        Vector3 stn = transform.position - lcwall.transform.position, dir = new Vector3(0, 0, 0);

        float z = stn.z * lcwall.transform.localScale.x;
        float x = stn.x * lcwall.transform.localScale.z;

        int iforce = 0, fforce = 2;

        if (z >= Math.Abs(x))
        {
            dir = new Vector3(fforce, 0, -1 * iforce);
        }
        else if (x > Math.Abs(z))
        {
            dir = new Vector3(-1 * iforce, 0, -1 * fforce);
        }
        else if (z < -1 * Math.Abs(x))
        {
            dir = new Vector3(-1 * fforce, 0, iforce);
        }
        else if (x < -1 * Math.Abs(z))
        {
            dir = new Vector3(iforce, 0, fforce);
        }

        dir = dir.normalized;

        if (Vector3.Dot(rBody.velocity, dir) > 0.6f)
        {
            dir = Vector3.zero;
        }

        Vector3 ifforce = stn.normalized * -0.1f;

        Vector3 finForce = dir;
        if (rBody.velocity.magnitude > 0.01f)
            finForce += ifforce;
        else
            finForce = -10 * ifforce;

        if (vw)
        {
            Debug.DrawLine(transform.position, lcwall.transform.position, Color.red);
            Debug.DrawLine(transform.position, transform.position + finForce * 5, Color.green);
        }

        return mass * (finForce + ifforce);
    }

    #endregion

    #region Group Behaviors

    #region Leader Following

    public Agent ldr;
    public void removeLeader(Agent a)
    {
        hasldr = false;
    }
    bool hasldr = false, isldr = false;

    public void setLeader(Agent a)
    {
        hasldr = true;
        ldr = a;
    }
    public void makeLeader()
    {
        isldr = true;
    }

    private Vector3 CalculateLeaderForce()
    {
        if (!hasldr)
        {
            return Vector3.zero;
        }

        Vector3 pdf = transform.position - ldr.transform.position;

        float dist = Math.Abs(pdf.magnitude) - radius - ldr.radius;
        float forceMult = 2 / dist - 1;

        if (Vector3.Dot(rBody.velocity, pdf.normalized) < -0.7f && forceMult < 0)
        {
            return Vector3.zero;
        }

        if (Vector3.Dot(rBody.velocity, pdf.normalized) > 0.7f && forceMult > 0)
        {
            return Vector3.zero;
        }

        return pdf.normalized * mass * forceMult;
    }
    #endregion

    #region Crowd Following

    private Vector3 CalculateCrowdForce()
    {
        Vector3 svel = rBody.velocity;
        foreach (GameObject agt in pn)
        {
            svel += agt.GetComponent<Rigidbody>().velocity;
        }

        return mass * ((svel / (pn.Count + 1)) - rBody.velocity);
    }

    #endregion

    #endregion

    #endregion
}
