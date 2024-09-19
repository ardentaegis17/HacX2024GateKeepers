using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LaunchControls : MonoBehaviour
{
    public GameObject projectile;
    public GameObject launchSite;
    public GameObject landingTarget;
    public float yMax;
    private float g; // gravity
    private float y_0;
    private float x;
    private float z;

    void Start()
    {
        // Set parameters that will determine time of flight and velocities required for launch
        g = 9.8f;
        y_0 = projectile.transform.position.y - landingTarget.transform.position.y;
        x = landingTarget.transform.position.x - launchSite.transform.position.x;
        z = landingTarget.transform.position.z - launchSite.transform.position.z;
    }

    void Update()
    {
        // If left-mouse button is clicked, launch projectile
        if (Input.GetMouseButtonDown(0))
        {
            LaunchBall(projectile);
        }
    }

    void LaunchBall(GameObject launchObject)
    {
        // Launch projectile using rigid body
        
        Rigidbody projectileBody = launchObject.GetComponent<Rigidbody>();
        projectileBody.transform.position = launchSite.transform.position;
        projectileBody.velocity = CalculateVelocity();

    }

    Vector3 CalculateVelocity()
    {   
        // Distance X and Z to target location
        Vector3 displacementXZ = new Vector3(x, 0, z);

        // Implement equations derived from kinematic analysis
        Vector3 velocityY = Vector3.up*Mathf.Sqrt(2*g*(yMax - y_0));
        Vector3 velocityXZ = displacementXZ/(Mathf.Sqrt(2*(yMax - y_0)/g) + Mathf.Sqrt(2*yMax/g));
        
        Vector3 velocity = velocityXZ + velocityY;
        return velocity;
    }

}