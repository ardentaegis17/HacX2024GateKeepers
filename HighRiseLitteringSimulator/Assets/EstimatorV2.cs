using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.ConstrainedExecution;
using UnityEngine;
using TMPro;

public class EstimatorV2 : MonoBehaviour
{
    public static float GRAVITY = 9.81f;

    // TODO:  access parameters from QuickStart UI

    public GameObject chosenObject;
    public Rigidbody rigidBody; 
    public string objectName; 
    public Vector3 landingTarget; 

    public Vector3 estimatedOrigin;
    public float throwVelocity;
    public float throwAngle; // Affects Y computation. Assume thrown at 45 degree angle (0.785 rad)
    public float facingAngle;// Affects XZ computation. Assume thrown at 45 degree angle (0.785 rad)
    
    public float windSpeed; // Affects X vector
    public int windDirection;
    public float dragCoefficient;

    // Discrete X coordinates, along the face of the HDB
    private static float[] xCoordinates = {30, 35, 40, 45, 50, 55, 60, 65, 70 };
    // Discrete Y coordinates, based on the different floors and height of parapet walls
    private static float[] yCoordinates = initializeYCoordinates();
    private float deltaT1 = 0.001f;
    public float surfaceArea;

    

    private Boolean isSimulating = false;
    // visually plot trajectory
    private LineRenderer lineRenderer;
    public Material lineMaterial;

    public TextMeshProUGUI estimatedOriginText;
    public TextMeshProUGUI closestLandingText;
    public TextMeshProUGUI initialVelocityText;
    public TextMeshProUGUI parametersText;

    void Start()
    {
        
        Time.fixedDeltaTime = 0.001f;

        chosenObject =  LauncherManager.chosenObject;
        chosenObject.AddComponent<Rigidbody>();
        rigidBody = chosenObject.GetComponent<Rigidbody>();
        objectName = QuickStartUI.thrownObject;
        landingTarget = QuickStartUI.landingPosition;
        throwVelocity = QuickStartUI.throwSpeed;
        windSpeed =  QuickStartUI.windSpeed;
        windDirection = QuickStartUI.windDirection;
        throwAngle = QuickStartUI.throwAngle;
        facingAngle = QuickStartUI.faceAngle;

        if(windDirection == 0){
            windSpeed *= -1;
        }

        switch(objectName)
        {
            case "Bottle":
                rigidBody.mass = 0.3f;
                dragCoefficient = 0.82f;
                surfaceArea = 0.0141f;
                break;
            case "Flowerpot":
                rigidBody.mass = 4.0f;
                dragCoefficient = 0.629f;
                surfaceArea = 0.15f;
                break;
            default:
                break;
        }

        estimatedOrigin = computeLikelyOrigin();

    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButtonDown(0) && !isSimulating)
        {
            launchObject(estimatedOrigin);
        }
    }
    private static float[] initializeYCoordinates()
    {
        float[] yCoordinates = new float[25];
        for (int i = 0; i < yCoordinates.Length; i++)
        {
            yCoordinates[i] = 6f + i * 3.6f + 0.25f;
        }
        return yCoordinates;
    }

    Vector3 computeLikelyOrigin()
    {
        Vector3 closestOrigin = Vector3.zero;
        Vector3 closestLandingPoint = Vector3.zero;
        Vector3 landingPoint = landingTarget;
        float smallestDistanceFromEndPoint = float.MaxValue;
        float initialVelocityX = computeVelocityX();
        float initialVelocityY = computeVelocityY();
        float initialVelocityZ = computeVelocityZ();

       

        Debug.Log("Throw Velocity: " + throwVelocity + " Throw Angle: " + throwAngle + " Face Angle: " + facingAngle);
        Debug.Log("initial Velocity: " + (initialVelocityX,initialVelocityY,initialVelocityZ));

        foreach (float initialX in xCoordinates)
        {
            foreach(float initialY in yCoordinates)
            {
                (float xDistance, float zDistance) = estimateTimeAndPositionWithDrag(initialVelocityX, initialVelocityY, initialVelocityZ, initialY);
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
        
        estimatedOriginText.text = $"Origin estimate: {closestOrigin}";
        //Debug.Log("Origin estimate = " + closestOrigin);
        closestLandingText.text = $"Closest landing point: {closestLandingPoint}";
        Debug.Log("Closest landing point: " + closestLandingPoint);
        initialVelocityText.text = $"Initial Velocity: {(initialVelocityX, initialVelocityY, initialVelocityZ)}";
        parametersText.text = $"Throw Speed: {throwVelocity} m/s \n Throw Angle: {throwAngle} radians \n Face Angle: {facingAngle} radians \n Wind Speed: {windSpeed} m/s \n Wind Direction: {(windDirection > 0 ? "Right" : "Left")}";

        return closestOrigin;
    }

    (float, float) estimateTimeAndPositionWithDrag(float initialVelocityX, float initialVelocityY, float initialVelocityZ, float initialY) {
        float yCoordinate = initialY;
        float deltaX = 0f;
        float deltaZ = 0f;
        float velocityX = initialVelocityX;
        float velocityY = initialVelocityY;
        float velocityZ = initialVelocityZ;
        float time = 0f;
        float deltaT = 0.001f;

        while (yCoordinate > 0) {
            float dragForceX = dragCoefficient * 1.225f * 0.5f * surfaceArea * velocityX * velocityX;
            float dragForceY = dragCoefficient * 1.225f * 0.5f * surfaceArea * velocityY * velocityY;
            float dragForceZ = dragCoefficient * 1.225f * 0.5f * surfaceArea * velocityZ * velocityZ;


            float accelerationX = -(dragForceX/rigidBody.mass) * Mathf.Sign(velocityX);
            float accelerationY = -GRAVITY - (dragForceY / rigidBody.mass) * Mathf.Sign(velocityY);
            float accelerationZ = -(dragForceZ/rigidBody.mass) * Mathf.Sign(velocityZ); 

            velocityX = velocityX + accelerationX * deltaT;
            velocityY = velocityY + accelerationY * deltaT;
            velocityZ = velocityZ + accelerationZ * deltaT;

            yCoordinate = yCoordinate + velocityY * deltaT;
            deltaX = deltaX + velocityX * deltaT;
            deltaZ = deltaZ + velocityZ * deltaT;

            time = time + deltaT;
        }

        return (deltaX, deltaZ);
    }

    void launchObject(Vector3 origin)
    {
        rigidBody.transform.position = origin;
        Vector3 initialVelocityVector = computeInitialVelocityVector();
        Debug.Log("Velocity vector: " + initialVelocityVector);
        rigidBody.velocity = initialVelocityVector;
        StartCoroutine(SimulateTrajectory(estimatedOrigin, initialVelocityVector));
    }

    IEnumerator SimulateTrajectory(Vector3 origin, Vector3 initialVelocityVector)
    {
        isSimulating = true;
        float time = 0f;
        float deltaT = deltaT1;
        Vector3 position = origin;
        Vector3 velocity = initialVelocityVector;
        // get lineRenderer component
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.positionCount = 10000;
        int i = 0;
        lineRenderer.SetPosition(i,position);
        // Set the material of the line
        lineRenderer.material = lineMaterial;
        // Set the width of the line
        lineRenderer.startWidth = 0.5f;
        lineRenderer.endWidth = 0.5f;

        while (position.y > 0)
        {
            position += velocity * deltaT;
            i++;
            lineRenderer.SetPosition(i, position);

            float dragForceX = dragCoefficient * 1.225f * 0.5f * surfaceArea* velocity.x * velocity.x;
            float dragForceY = dragCoefficient * 1.225f * 0.5f * surfaceArea * velocity.y * velocity.y;
            float dragForceZ = dragCoefficient * 1.225f * 0.5f * surfaceArea * velocity.z * velocity.z;


            float accelerationX = -(dragForceX / rigidBody.mass) * Mathf.Sign(velocity.x);
            float accelerationY = -GRAVITY - (dragForceY / rigidBody.mass) * Mathf.Sign(velocity.y);
            float accelerationZ = -(dragForceZ / rigidBody.mass) * Mathf.Sign(velocity.z);

            velocity.x = velocity.x + accelerationX * deltaT;
            velocity.y = velocity.y + accelerationY * deltaT;
            velocity.z = velocity.z + accelerationZ * deltaT;

            rigidBody.transform.position = position;

            time += deltaT;
            yield return new WaitForFixedUpdate();
        }
        isSimulating = false;
        //rigidBody.isKinematic = false;
    }

    Vector3 computeInitialVelocityVector()
    {
        return new Vector3(computeVelocityX(), computeVelocityY(), computeVelocityZ());
    }

    float computeVelocityX()
    {
        float throwAngleRad = throwAngle * Mathf.Deg2Rad;
        float facingAngleRad = facingAngle * Mathf.Deg2Rad;
        return throwVelocity * Mathf.Cos(throwAngleRad) * Mathf.Cos(facingAngleRad) + windSpeed; 
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
