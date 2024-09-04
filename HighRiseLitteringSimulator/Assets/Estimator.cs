using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.ConstrainedExecution;
using UnityEngine;

public class Estimator : MonoBehaviour
{
    public static float GRAVITY = 9.81f;
    public Vector3 landingPoint;
    public Vector3 estimatedOrigin;
    public float throwVelocity;
    public float throwAngle; // Affects Y computation
    public float facingAngle; // Affects XZ computation
    public Rigidbody rigidBody;

    // Discrete X coordinates, along the face of the HDB
    public float[] xCoordinates = { 0, 24, 32, 40, 48, 56, 64, 72 };
    // Discrete Y coordinates, based on the different floors and height of parapet walls
    public float[] yCoordinates = new float[25];

    // Start is called before the first frame update
    void Start()
    {
        updateYCoordinates();
        estimatedOrigin = computeLikelyOrigin();
        Debug.Log(estimatedOrigin);
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            launchObject(estimatedOrigin);
        }
    }

    void updateYCoordinates()
    {
        for (int i = 0; i < yCoordinates.Length; i++)
        {
            yCoordinates[i] = 6f + i * 3.6f;
        }
    }

    Vector3 computeLikelyOrigin()
    {
        Vector3 closestOrigin = Vector3.zero;
        Vector3 closestLandingPoint = Vector3.zero;
        float smallestDistanceFromEndPoint = float.MaxValue;
        float initialVelocityX = computeVelocityX();
        float initialVelocityY = computeVelocityY();
        float initialVelocityZ = computeVelocityZ();

        foreach (float initialX in xCoordinates)
        {
            foreach(float initialY in yCoordinates)
            {
                float timeOfFlight = (initialVelocityY + Mathf.Sqrt(initialVelocityY * initialVelocityY + 2 * GRAVITY * initialY)) / GRAVITY;
                float xDistance = initialVelocityX * timeOfFlight;
                float zDistance = initialVelocityZ * timeOfFlight;

                Vector3 estimatedLanding = new Vector3(initialX + xDistance, 0, zDistance);
                float distanceToActualLanding = Vector3.Distance(estimatedLanding, landingPoint);

                if (distanceToActualLanding < smallestDistanceFromEndPoint)
                {
                    smallestDistanceFromEndPoint = distanceToActualLanding;
                    closestOrigin = new Vector3(initialX, initialY, 0);
                    closestLandingPoint = estimatedLanding;
                }
            }
        }
        Debug.Log("Closest landing point: " + closestLandingPoint);
        return closestOrigin;
    }

    void launchObject(Vector3 origin)
    {
        rigidBody.transform.position = origin;
        Vector3 initialVelocityVector = computeInitialVelocityVector();
        Debug.Log("Velocity vector: " + initialVelocityVector);
        rigidBody.velocity = initialVelocityVector;
    }

    Vector3 computeInitialVelocityVector()
    {
        return new Vector3(computeVelocityX(), computeVelocityY(), computeVelocityZ());
    }

    float computeVelocityX()
    {
        float throwAngleRad = throwAngle * Mathf.Deg2Rad;
        float facingAngleRad = facingAngle * Mathf.Deg2Rad;
        return throwVelocity * Mathf.Cos(throwAngleRad) * Mathf.Cos(facingAngleRad);
    }

    float computeVelocityY()
    {
        float throwAngleRad = throwAngle * Mathf.Deg2Rad;
        float facingAngleRad = facingAngle * Mathf.Deg2Rad;
        return throwVelocity * Mathf.Sin(throwAngleRad);
    }

    float computeVelocityZ()
    {
        float throwAngleRad = throwAngle * Mathf.Deg2Rad;
        float facingAngleRad = facingAngle * Mathf.Deg2Rad;
        return throwVelocity * Mathf.Cos(throwAngleRad) * Mathf.Sin(facingAngleRad);
    }


}
