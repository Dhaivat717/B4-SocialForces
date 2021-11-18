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
    private bool  vn = true;
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
        if(isldr) {
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

        if (vp)
        {
            if (p.Count > 0)
            {
                Debug.DrawLine(transform.position, p[0], Color.green);
            }
            for (int i = 0; i < p.Count - 1; i++)
            {
                Debug.DrawLine(p[i], p[i + 1], Color.yellow);
            }
        }

        if (vn)
        {
            foreach (var neighbor in pn)
            {
                Debug.DrawLine(transform.position, neighbor.transform.position, Color.yellow);
            }
        }

        #endregion
    }

    #region Public Functions

    public void ComputePath(Vector3 destination)
    {
        nam.enabled = true;
        var nmPath = new NavMeshPath();
        nam.CalculatePath(destination, nmPath);
        p = nmPath.corners.Skip(1).ToList();
        nam.enabled = false;

    }

    public Vector3 GetVelocity()
    {
        return rBody.velocity;
    }

    #endregion

    #region Part 1

    private Vector3 ComputeForce()
    {
        var force = Vector3.zero;
        force += CalculateAgentForce();
        if(hasldr) {
            force += CalculateLeaderForce();
        }
        

        wf = Vector3.zero;
        if (force != Vector3.zero)
        {
            return force.normalized * Mathf.Min(force.magnitude, Parameters.maxSpeed);
        } else
        {
            return Vector3.zero;
        }
        
    }
    
    private Vector3 CalculateGoalForce()
    {
        Vector3 velocity = GetVelocity();
        Vector3 direction = (p[0] - transform.position).normalized;

        Vector3 force = mass*(DESIRED_VELOCITY*direction-velocity);

        return force;
    }

    private Vector3 CalculateAgentForce()
    {
        Vector3 force = Vector3.zero;
        int count = pn.RemoveWhere(g => !g.activeSelf);
       
        foreach(var agent in pn){
            float distance = -Vector3.Distance(transform.position, agent.transform.position);
            Vector3 direction = (transform.position - agent.transform.position).normalized;

            float agentRadii = agent.gameObject.GetComponent<Agent>().radius + radius;
            float agentOverLap = Math.Max(agentRadii - Vector3.Distance(transform.position, agent.transform.position), 0);

            force += Parameters.A * Mathf.Exp(distance / Parameters.B) * direction;

            force += agentOverLap * Parameters.k * distance *direction;

            Vector3 tang = Vector3.Cross(Vector3.up, direction);
            force += agentOverLap * Parameters.Kappa * Vector3.Dot(rBody.velocity - agent.gameObject.GetComponent<Rigidbody>().velocity, tang) * tang;

        }

        return force;
    }
    
    private void CalculateWallForce(Collision collision)
    {
        Vector3 force = Vector3.zero;
        Vector3 direction = -(collision.contacts[0].point - transform.position).normalized;
        float magnitude = 1.0f;

        wf += direction*magnitude*mass;
        return;
    }

    public void ApplyForce()
    {
        var force = ComputeForce();
        force.y = 0;

        rBody.AddForce(force * 10, ForceMode.Force);
    }

    public void OnTriggerEnter(Collider other)
    {
        if (AgentManager.IsAgent(other.gameObject))
        {
            pn.Add(other.gameObject);
        }
    }
    
    public void OnTriggerExit(Collider other)
    {
        if (pn.Contains(other.gameObject))
        {
            pn.Remove(other.gameObject);
        }
    }

    public void OnCollisionEnter(Collision collision)
    {
        if(collision.gameObject.tag == "wall") {
            lastCollidedWall = collision.gameObject;
        }

        if(WallManager.IsWall(collision.gameObject))
        {
            CalculateWallForce(collision);
        }
    }

    public void OnCollisionExit(Collision collision)
    {
        
    }

    #endregion

    #region Part 2

    #region Single-Agent Behaviors

        GameObject lastCollidedWall = null;
        float maxWallDist;

        private Vector3 CalculateWallFollowForce() {
            if(lastCollidedWall == null) {
                return Vector3.zero;
            }

            Vector3 separation = transform.position - lastCollidedWall.transform.position, direction = new Vector3(0,0,0);

            
            int inwardF = 0, forwardF = 2;
            float x = separation.x * lastCollidedWall.transform.localScale.z;
            float z = separation.z * lastCollidedWall.transform.localScale.x;



            if(z >= Math.Abs(x)) {
                direction = new Vector3(forwardF,0,-1*inwardF);
            }
            else if(x > Math.Abs(z)) {
                direction = new Vector3(-1*inwardF,0,-1*forwardF);
            }
            else if(z < -1*Math.Abs(x)) {
                direction = new Vector3(-1*forwardF,0,inwardF);
            }
            else if(x < -1*Math.Abs(z)) {
                direction = new Vector3(inwardF,0,forwardF);
            }

            direction = direction.normalized;
            float directionalVelocity = Vector3.Dot(rBody.velocity, direction);
            if(directionalVelocity > 0.6f) {
                direction = Vector3.zero;
            }

            Vector3 inF = separation.normalized * -0.1f;

            Vector3 finalForce = direction;
            if(rBody.velocity.magnitude > 0.01f)
                finalForce += inF;
            else
                finalForce = -10*inF;
            
            if(vw) {
                Debug.DrawLine(transform.position, lastCollidedWall.transform.position, Color.red);
                Debug.DrawLine(transform.position, transform.position+finalForce*5, Color.green);
            }

            return mass*(finalForce + inF);
        }


        private Vector3 CalculateSpiralForce(){
            Vector3 toCenter = Vector3.zero - new Vector3(transform.position.x, 0, transform.position.z);
            Vector3 tang = Vector3.Cross(toCenter, Vector3.up);

            Vector3 newForce = tang;
            newForce += toCenter;

            Debug.DrawLine(transform.position, transform.position+tang, Color.magenta);
            Debug.DrawLine(transform.position, transform.position+toCenter, Color.green);

            return newForce * mass;
        }

    #endregion

    #region Group Behaviors

        #region Crowd Following

        private Vector3 CalculateCrowdForce() {
            Vector3 sumVelocity = rBody.velocity;
            foreach(GameObject agent in pn) {
                
                sumVelocity += agent.GetComponent<Rigidbody>().velocity;

            }

            Vector3 desiredVelocity = sumVelocity / (pn.Count + 1);
            
            return mass * (desiredVelocity - rBody.velocity);
        }

        #endregion

        #region Leader Following

        public Agent ldr;
        bool hasldr = false, isldr = false; 

        public void setLeader(Agent a) {
            hasldr = true;
            ldr = a;
        }
        public void removeLeader(Agent a) {
            hasldr = false;
        }
        public void makeLeader() {
            isldr = true;
        }
        private Vector3 CalculateLeaderForce() {
            if(!hasldr) {
                return Vector3.zero;
            }
            
            Vector3 posDif = transform.position - ldr.transform.position;

            float dist = Math.Abs(posDif.magnitude) - radius - ldr.radius;
            float forceMult = 2 / dist - 1;

            if(Vector3.Dot(rBody.velocity, posDif.normalized) > 0.7f && forceMult > 0) {
                return Vector3.zero;
            }
            if(Vector3.Dot(rBody.velocity, posDif.normalized) < -0.7f && forceMult < 0) {
                return Vector3.zero;
            }

            return posDif.normalized * mass * forceMult;
        }
        #endregion

    #endregion

    

    #endregion
}
