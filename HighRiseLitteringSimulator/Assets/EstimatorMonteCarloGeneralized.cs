using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using MathNet.Numerics;
using MathNet.Numerics.Distributions;
using UnityEngine;
using System.Threading.Tasks;
using Unity.VisualScripting;
using TMPro;


public class EstimatorMonteCarloGeneralized : MonoBehaviour
{
    /** 
     * Static variables
     * 
     * Gravitational force constant
     * Lambda for exponential decay function
     */
    public static float GRAVITY = 9.81f;
    public double LAMBDA = 5.0d;
    public double mass;

    /**
     * Simulation parameters : Type
     * 
     * Delta time for simulation: Float representing time between each step of the simulation
     * Surface area: Double representing the surface area of the object dropped
     * Drag coefficient: Double representing the drag coefficient of the object dropped
     * isSimulating: Boolean
     * Number of iterations: Integer representing the number of Monte Carlo simulations performed
     * 
     */
    public float deltaTSimulation = 0.001f;
    public double surfaceArea;
    public double dragCoefficient;
    private Boolean isSimulating = false;
    public int numOfIterations;
    public GameObject chosenObject;
    public Rigidbody rigidBody;
    public string objectName; 
    private double rmass = 1.0d; //For testing purposes
    public Vector3 landingPointVector;


    /**
     * Initial customizable parameters : Type
     * 
     * Throw velocity shape
     * Throw velocity rate
     * Throw angle mean
     * Throw angle standard deviation
     * Facing angle mean
     * Facing angle standard deviation
     * Wind speed shape
     * Wind speed scale
     * Simulate global optimum or mean best
     * Sets all values to default params
     * Use presets
     */
    public double throwVelocityDistributionShape;
    public double throwVelocityDistributionRate;
    public double throwAngleDistributionMean;
    public double throwAngleDistributionSD;
    public double facingAngleDistributionMean;
    public double facingAngleDistributionSD;
    public double windSpeedDistributionShape;
    public double windSpeedDistributionScale;
    public String globalOptimumOrMeanBest;
    public Boolean useDefaultDistributions;
    public String presets;

    /** 
     * Initial physical parameters : Type
     * 
     * Landing point: 3D position vector
     * Throw velocity: Gamma random variable with mean 10 m/s and variance 4 m^2/s^2 (Parameters: shape = 50, rate = 5)
     * Throw angle: Normal random variable with mean 0 radians and variance PI/6 radians
     * Facing angle: Normal random variable with mean PI/2 radians and variance PI/6 radians
     * Wind speed: Weibull random variable with mean 3.4 m/s (Parameters: shape = 2.037, scale factor = 3.835, using Pandan Gardens)
     * Wind direction: Bernoulli random variable with p = 0.5
     * 
     * X coordinates: Array of doubles representing possible values of the X coordinate
     * Y coordinates: Array of doubles representing possible values of the Y coordinate
     * Points: Array of tuples representing possible points
     * 
     */
    public (double, double, double) landingPoint;
    private Gamma throwVelocity;
    private Normal throwAngle;
    private Normal facingAngle;
    private Weibull windSpeed; 
    private Bernoulli windDirection = new Bernoulli(0.5);
    public static double[] xCoordinates = {30, 35, 40, 45, 50, 55, 60, 65, 70 };
    public static double[] yCoordinates = initializeYCoordinates();
    private List<(double, double, double)> points = cartesianProductForTwoSets(xCoordinates, yCoordinates);
    
    // visually plot trajectory
    private LineRenderer lineRenderer;
    public Material lineMaterial;

    // display computed results
    public TextMeshProUGUI estimatedOrigin1;
    public TextMeshProUGUI estimatedOrigin2;
    public TextMeshProUGUI estimatedOrigin3;
    public TextMeshProUGUI globalOrigin;
    public TextMeshProUGUI localOrigin;

    public List<TextMeshProUGUI> textMeshProList;

    /**
     * Computed results : Type
     * 
     * k: Integer representing the number of origins to return
     * Estimated origins: Array of tuples representing the k most likely origin points based on a distance weighted score given an terminal point
     * Weight adjusted scores: Dictionary containing points and their associated distance weighted scores
     * Stored points and parameters: Dictionary containing most likely origin points and the mean of the sample parameters
     * Global one-off optimum, distance from actual landing point and parameters: The very best throw made across all iterations
     * 
     */
    public int k;
    public List<((double, double, double), double, (double, double, double, double, int))> estimatedOrigins;
    public ConcurrentDictionary<(double, double, double), double> weightAdjustedScores = new ConcurrentDictionary<(double, double, double), double>();
    public ConcurrentDictionary<(double, double, double), ((double tv, double ta, double fa, double ws, double wd) parameters, double ed)> localOptima = new ConcurrentDictionary<(double, double, double), ((double tv, double ta, double fa, double ws, double wd) parameters, double ed)>();
    public ConcurrentDictionary<(double, double, double), (double tv, double ta, double fa, double ws, int count)> storedPointsAndParams = new ConcurrentDictionary<(double, double, double), (double, double, double, double, int)>();
    public ((double x, double y, double z) likelyOrigin, (double lx, double ly, double lz) estimatedLanding, (double tv, double ta, double fa, double ws, double wd) parameters, double ed) globalOptimum = new((0, 0, 0), (0, 0, 0), (0, 0, 0, 0, 0), Double.MaxValue);

    /**
     * Unity engine functions : Purpose
     * 
     * Start: Starts the program and initializes the environment
     * Initialize parameters: Initializes parameters
     * Update: Called once per frame to update the environment
     */
    void Start()
    {
        chosenObject = LauncherManager.chosenObject;
        chosenObject.AddComponent<Rigidbody>();
        rigidBody = chosenObject.GetComponent<Rigidbody>();
        useDefaultDistributions = SandboxUI.useDefaultDistribution;
        globalOptimumOrMeanBest = SandboxUI.globalOptimumOrMeanBest;
        throwVelocityDistributionShape = SandboxUI.throwVelocityDistributionShape;
        throwVelocityDistributionRate = SandboxUI.throwVelocityDistributionRate;
        throwAngleDistributionMean = SandboxUI.throwAngleDistributionMean;
        throwAngleDistributionSD = SandboxUI.throwAngleDistributionSD;
        facingAngleDistributionMean = SandboxUI.facingAngleDistributionMean;
        facingAngleDistributionSD = SandboxUI.facingAngleDistributionSD;
        windSpeedDistributionShape = SandboxUI.windSpeedDistributionShape;
        windSpeedDistributionScale = SandboxUI.windSpeedDistributionScale;
        
        objectName = SandboxUI.thrownObject;
        landingPointVector = SandboxUI.landingPosition;

        textMeshProList = new List<TextMeshProUGUI> {estimatedOrigin1, estimatedOrigin2, estimatedOrigin3, globalOrigin, localOrigin};

        Time.fixedDeltaTime = deltaTSimulation;
        initializeParameters();
        Debug.Log(throwVelocity.Shape + ", " + throwVelocity.Rate + ", "+ throwAngle.Mean + ", " + throwAngle.StdDev + ", " + facingAngle.Mean + ", " + facingAngle.StdDev + ", " + windSpeed.Shape + ", " + windSpeed.Scale);
        landingPoint = convertVectorToTuple(landingPointVector);
        mass = rigidBody.mass;
        runMonteCarlo(numOfIterations);
        for (int i = 0; i < estimatedOrigins.Count; i++)
        {
            // Debug.Log("Likely origin: " + estimatedOrigins[i].Item1);
            textMeshProList[i].text = $"Likely origin {i+1}: {estimatedOrigins[i].Item1} \n  Score: {Math.Round(estimatedOrigins[i].Item2,2)} \n # Iterations: {estimatedOrigins[i].Item3.Item5}"; 
            // Debug.Log("Cumulative distance adjusted score (higher is better): " + estimatedOrigins[i].Item2);
            // Debug.Log("Number of iterations this was the most likely origin: " + estimatedOrigins[i].Item3.Item5);
        }

        (double, double, double) roundedLanding = new (
            Math.Round(globalOptimum.estimatedLanding.Item1,2),
            Math.Round(globalOptimum.estimatedLanding.Item2,2),
            Math.Round(globalOptimum.estimatedLanding.Item3,2)
        );
        textMeshProList[3].text = $"Global optimum origin: \n {globalOptimum.likelyOrigin} \n Estimated landing: \n {roundedLanding} \n Throw Velocity: {Math.Round(globalOptimum.parameters.Item1,2)} m/s \n Throw Angle: {Math.Round(globalOptimum.parameters.Item2,2)} radians \n Face Angle: {Math.Round(globalOptimum.parameters.Item3,2)} radians Wind Speed: {Math.Round(globalOptimum.parameters.Item4,2)} m/s Wind Direction: {(globalOptimum.parameters.Item5 > 0 ? "Right" : "Left")} \n Euclidean distance from actual landing point: {Math.Round(globalOptimum.ed,2)}";
        // Debug.Log("Global optimum origin: " + globalOptimum.likelyOrigin + ", Estimated landing: " + globalOptimum.estimatedLanding + ", Parameters used: " + globalOptimum.parameters + ", Euclidean distance from actual landing point: " + globalOptimum.ed);
        if (localOptima.TryGetValue(estimatedOrigins[0].Item1, out ((double tv, double ta, double fa, double ws, double wd) parameters, double ed) optima))
        {
            // textMeshProList[4].text = $"Local optimum origin: {estimatedOrigins[0].Item1} \n Parameters used: \n Throw Velocity: {optima.parameters.tv}  Throw Angle: {optima.parameters.ta} \n Facing Angle: {optima.parameters.fa} Wind Speed: {optima.parameters.ws} \n Wind direction: {optima.parameters.wd} Error: {optima.ed}";
            // Debug.Log("Local optimum origin: " + estimatedOrigins[0].Item1 + "Parameters used: Throw velocity: " + optima.parameters.tv + ", Throw angle: " + optima.parameters.ta + ", Facing angle: " + optima.parameters.fa + ", Wind speed: " + optima.parameters.ws + ", Wind direction: " + optima.parameters.wd + ", Error: " + optima.ed);
        }
    }

    void Update()
    {
        if (Input.GetMouseButtonDown(0) && !isSimulating)
        {
            switch (globalOptimumOrMeanBest)
            {
                case "Global optima":
                    launchObjectOptima(globalOptimum);
                    break;
                case "Most likely optima":
                    if (localOptima.TryGetValue(estimatedOrigins[0].Item1, out ((double tv, double ta, double fa, double ws, double wd) parameters, double ed) optima))
                    {
                        launchObjectMostLikelyOptima(optima.parameters.tv, optima.parameters.ta, optima.parameters.fa, optima.parameters.ws, optima.parameters.wd, optima.ed, estimatedOrigins[0].Item1);
                    }
                        
                    break;
                default:
                    launchObjectMean(estimatedOrigins[0]);
                    break;
            }
        }
    }

    void initializeParameters()
    {
        if (useDefaultDistributions)
        {
            throwVelocity = new Gamma(4d, 0.85d);
            throwAngle = new Normal(0d, Math.PI / 6d);
            facingAngle = new Normal((Math.PI) / 2d, Math.PI / 6d);
            windSpeed = new Weibull(2.037, 3.835);
            
        } else
        {


            Debug.Log($"velocity dist shape: {throwVelocityDistributionShape}" + $" velocity dist rate: {throwVelocityDistributionRate}");
            Debug.Log($"wind dist shape: {windSpeedDistributionShape}" + $" wind dist scale: {windSpeedDistributionScale}");

            throwVelocity = new Gamma(throwVelocityDistributionShape, throwVelocityDistributionRate);
            throwAngle = new Normal(throwAngleDistributionMean, throwAngleDistributionSD);
            facingAngle = new Normal(facingAngleDistributionMean, facingAngleDistributionSD);
            windSpeed = new Weibull(windSpeedDistributionShape, windSpeedDistributionScale);
        }
         switch(objectName)
        {
            case "BottlePreset":
                rigidBody.mass = 0.3f;
                dragCoefficient = 0.82d;
                surfaceArea = 0.0141d;
                landingPointVector = new Vector3(50, 0, 12);
                break;
            case "FlowerpotPreset":
                rigidBody.mass = 1.0f;
                dragCoefficient = 0.82d;
                surfaceArea = 0.15d;
                landingPointVector = new Vector3(50, 0, 4);
                break;
            case "Bottle":
                rigidBody.mass = 0.3f;
                dragCoefficient = 0.82d;
                surfaceArea = 0.0141d;
                break;
            case "Flowerpot":
                rigidBody.mass = 4.0f;
                dragCoefficient = 0.82d;
                surfaceArea = 0.15d;
                break;
            default:
                break;
        }

    }

    /** 
     * Initialization functions : Purpose
     * 
     * Initialize Y Coordinates: Initializes the Y coordinates
     * Cartesian product for two sets: Returns the cartesian product of 2 sets of doubles
     * 
     */
    private static double[] initializeYCoordinates()
    {
        double[] yCoordinates = new double[25];
        for (int i = 0; i < yCoordinates.Length; i++)
        {
            yCoordinates[i] = 6d + i * 3.6d + 0.25d;
        }
        return yCoordinates;
    }

    private static List<(double, double, double)> cartesianProductForTwoSets(double[] setOne, double[] setTwo)
    {
        List<(double, double, double)> productSet = new List<(double, double, double)>();

        foreach (double x in setOne)
        {
            foreach (double y in setTwo)
            {
                productSet.Add((x, y, 0d));
            }
        }

        return productSet;
    }

    /**
     *  Functions for unity 3D simulation
     *  
     *  Launch Object: Launches a rigidbody using the most likely origin with the sample mean parameters
     */
    void launchObjectMean(((double, double, double) estimatedOrigin, double, (double throwVelocity, double throwAngle, double facingAngle, double windSpeed, int) parameters) estimatedOriginWithMeanParameters)
    {
        double trueWindSpeed = estimatedOriginWithMeanParameters.estimatedOrigin.Item1 < landingPoint.Item1 ? estimatedOriginWithMeanParameters.parameters.windSpeed : -estimatedOriginWithMeanParameters.parameters.windSpeed;
        Vector3 estimatedOriginV = new Vector3((float)estimatedOriginWithMeanParameters.estimatedOrigin.Item1, (float)estimatedOriginWithMeanParameters.estimatedOrigin.Item2, (float)estimatedOriginWithMeanParameters.estimatedOrigin.Item3);
        rigidBody.transform.position = estimatedOriginV;
        (double, double, double) initialVelocityVector =
            computeInitialVelocityVector(estimatedOriginWithMeanParameters.parameters.throwVelocity, estimatedOriginWithMeanParameters.parameters.throwAngle, estimatedOriginWithMeanParameters.parameters.facingAngle, estimatedOriginWithMeanParameters.parameters.windSpeed);
        Vector3 initialVelocityVectorV = new Vector3((float)initialVelocityVector.Item1, (float)initialVelocityVector.Item2, (float)initialVelocityVector.Item3);
        StartCoroutine(simulateTrajectoryUnity(estimatedOriginV, initialVelocityVectorV));
    }

    void launchObjectMostLikelyOptima(double tv, double ta, double fa, double ws, double wd, double ed, (double, double, double) point)
    {
        double trueWindSpeed = (wd == 1) ? ws * 1 : ws * -1;
        Vector3 estimatedOriginV = new Vector3((float)point.Item1, (float)point.Item2, (float)point.Item3);
        rigidBody.transform.position = estimatedOriginV;
        (double, double, double) initialVelocityVector = computeInitialVelocityVector(tv, ta, fa, trueWindSpeed);
        Vector3 initialVelocityVectorV = new Vector3((float) initialVelocityVector.Item1, (float) initialVelocityVector.Item2, (float)initialVelocityVector.Item3);
        StartCoroutine(simulateTrajectoryUnity(estimatedOriginV, initialVelocityVectorV));
    }

    void launchObjectOptima(((double x, double y, double z) likelyOrigin, (double lx, double ly, double lz) estimatedLanding, (double tv, double ta, double fa, double ws, double wd) parameters, double ed) globalOptimaWithParameters)
    {
        double trueWindSpeed = globalOptimaWithParameters.parameters.ws * globalOptimaWithParameters.parameters.wd;
        Vector3 estimatedOriginV = new Vector3((float)globalOptimaWithParameters.likelyOrigin.x, (float)globalOptimaWithParameters.likelyOrigin.y, (float)globalOptimaWithParameters.likelyOrigin.z);
        rigidBody.transform.position = estimatedOriginV;
        (double, double, double) initialVelocityVector = computeInitialVelocityVector(globalOptimaWithParameters.parameters.tv, globalOptimaWithParameters.parameters.ta, globalOptimaWithParameters.parameters.fa, trueWindSpeed);
        Vector3 initialVelocityVectorV = new Vector3((float)initialVelocityVector.Item1, (float)initialVelocityVector.Item2, (float)initialVelocityVector.Item3);
        StartCoroutine(simulateTrajectoryUnity(estimatedOriginV, initialVelocityVectorV));
    }

    IEnumerator simulateTrajectoryUnity(Vector3 estimatedOrigin, Vector3 initialVelocityVector)
    {
        isSimulating = true;
        float time = 0f;
        Vector3 position = estimatedOrigin;
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
            position += velocity * deltaTSimulation;
            i++;
            lineRenderer.SetPosition(i, position);

            float dragForceX = (float)dragCoefficient * 1.225f * 0.5f * (float)surfaceArea * velocity.x * velocity.x;
            float dragForceY = (float)dragCoefficient * 1.225f * 0.5f * (float)surfaceArea * velocity.y * velocity.y;
            float dragForceZ = (float)dragCoefficient * 1.225f * 0.5f * (float)surfaceArea * velocity.z * velocity.z;

            float accelerationX = -(dragForceX / rigidBody.mass) * Mathf.Sign(velocity.x);
            float accelerationY = -GRAVITY - (dragForceY / rigidBody.mass) * Mathf.Sign(velocity.y);
            float accelerationZ = -(dragForceZ / rigidBody.mass) * Mathf.Sign(velocity.z);

            velocity.x = velocity.x + accelerationX * deltaTSimulation;
            velocity.y = velocity.y + accelerationY * deltaTSimulation;
            velocity.z = velocity.z + accelerationZ * deltaTSimulation;

            rigidBody.transform.position = position;

            time += deltaTSimulation;
            yield return new WaitForFixedUpdate();
        }
        isSimulating = false;
    }



    /**
     * Functions for Monte Carlo simulations : Purpose
     * 
     * Run Monte Carlo: Runs a specified number of Monte Carlo simulations
     * Generate origin list: Generates the list of k most likely origins
     */
    void runMonteCarlo(int iterationCount)
    {
        Parallel.For(0, iterationCount, i =>
        {
            computeLikelyOrigin();
        });

        generateOutputOriginList(k, iterationCount);
    }

    void generateOutputOriginList(int k, int iterationCount)
    {
        List<((double, double, double) point, double score, (double, double, double, double, int) meanParams)> result = new List<((double, double, double), double, (double, double, double, double, int))>();
        var newList = weightAdjustedScores.OrderByDescending(entry => entry.Value).ToList();
        for (int i = 0; i < Math.Min(k, newList.Count); i++)
        {
            (double, double, double, double, int count) meanParams = storedPointsAndParams[newList[i].Key];
            meanParams.Item1 = meanParams.Item1 / (double)meanParams.Item5;
            meanParams.Item2 = meanParams.Item2 / (double)meanParams.Item5;
            meanParams.Item3 = meanParams.Item3 / (double)meanParams.Item5;
            meanParams.Item4 = meanParams.Item4 / (double)meanParams.Item5;
            result.Add((newList[i].Key, newList[i].Value, meanParams));
        }
        estimatedOrigins = result;

    }


    /**
     * Functions for a single Monte Carlo simulation : Purpose
     * 
     * Compute likely origin: Computes the most likely origin given initial parameters
     * Simulate trajectory: Simulates the trajectory of the thrown object
     */
    void computeLikelyOrigin()
    {
        double throwVelocitySample = throwVelocity.Sample();
        double throwAngleSample = throwAngle.Sample();
        double facingAngleSample = facingAngle.Sample();
        double windSpeedSample = windSpeed.Sample();
        int windDirectionSample = windDirection.Sample();
        (double velocityX, double velocityY, double velocityZ) initialVelocityVector;
        if (windDirectionSample == 1)
        {
            initialVelocityVector =
                computeInitialVelocityVector(throwVelocitySample, throwAngleSample, facingAngleSample, windSpeedSample);
        }
        else
        {
            initialVelocityVector =
                computeInitialVelocityVector(throwVelocitySample, throwAngleSample, facingAngleSample, -1 * windSpeedSample);
        }


        double smallestDistanceFromActualLandingPoint = double.MaxValue;
        (double, double, double) likelyOrigin = (0d, 0d, 0d);
        (double, double, double) closestLandingPoint = (0d, 0d, 0d);

        foreach ((double x, double y, double z) point in points)
        {
            (double xDistance, double zDistance) =
                simulateTrajectory(initialVelocityVector.velocityX, initialVelocityVector.velocityY, initialVelocityVector.velocityZ, point.y);

            (double landingX, double landingY, double landingZ) estimatedLanding = (point.x + xDistance, 0, point.z + zDistance);
            double distanceFromActualLandingPoint = compute3DEuclideanDistance(estimatedLanding, landingPoint);
            if (distanceFromActualLandingPoint < smallestDistanceFromActualLandingPoint)
            {
                smallestDistanceFromActualLandingPoint = distanceFromActualLandingPoint;
                likelyOrigin = point;
                closestLandingPoint = estimatedLanding;

            }

        }

        double weightAdjustedScore = computeWeightAdjustedScore(smallestDistanceFromActualLandingPoint, LAMBDA);
        weightAdjustedScores.AddOrUpdate(likelyOrigin, weightAdjustedScore, (key, oldValue) => oldValue + weightAdjustedScore);



        storedPointsAndParams.AddOrUpdate(likelyOrigin,
            (throwVelocitySample, throwAngleSample, facingAngleSample, windSpeedSample, 1),
            (key, oldParams) =>
            (
            oldParams.tv + throwVelocitySample,
            oldParams.ta + throwAngleSample,
            oldParams.fa + facingAngleSample,
            (windDirectionSample == 1) ? oldParams.ws + windSpeedSample : oldParams.ws - windSpeedSample,
            oldParams.count + 1
            ));

        if (!localOptima.ContainsKey(likelyOrigin))
        {
            localOptima.TryAdd(likelyOrigin, ((throwVelocitySample, throwAngleSample, facingAngleSample, windSpeedSample, windDirectionSample), smallestDistanceFromActualLandingPoint));
        }

        if (smallestDistanceFromActualLandingPoint < localOptima.GetValueOrDefault(likelyOrigin).ed)
        {
            localOptima.TryUpdate(likelyOrigin, ((throwVelocitySample, throwAngleSample, facingAngleSample, windSpeedSample, windDirectionSample), smallestDistanceFromActualLandingPoint), localOptima.GetValueOrDefault(likelyOrigin));
        }

        if (smallestDistanceFromActualLandingPoint < globalOptimum.ed)
        {

            globalOptimum.likelyOrigin = likelyOrigin;
            globalOptimum.estimatedLanding = closestLandingPoint;
            double windDirection = (windDirectionSample == 1) ? 1 : -1;
            globalOptimum.parameters = (throwVelocitySample, throwAngleSample, facingAngleSample, windSpeedSample, windDirection);
            globalOptimum.ed = smallestDistanceFromActualLandingPoint;
        }

    }

    (double, double) simulateTrajectory(double initialVelocityX, double initialVelocityY, double initialVelocityZ, double initialY)
    {
        double deltaX = 0d;
        double deltaZ = 0d;
        double yCoordinate = initialY;
        double velocityX = initialVelocityX;
        double velocityY = initialVelocityY;
        double velocityZ = initialVelocityZ;
        double time = 0d;

        while (yCoordinate > 0)
        {
            double dragForceX = dragCoefficient * 1.225f * 0.5f * surfaceArea * velocityX * velocityX;
            double dragForceY = dragCoefficient * 1.225f * 0.5f * surfaceArea * velocityY * velocityY;
            double dragForceZ = dragCoefficient * 1.225f * 0.5f * surfaceArea * velocityZ * velocityZ;

            double accelerationX = -(dragForceX / mass) * Math.Sign(velocityX);
            double accelerationY = -GRAVITY - (dragForceY / mass) * Math.Sign(velocityY);
            double accelerationZ = -(dragForceZ / mass) * Math.Sign(velocityZ);

            velocityX = velocityX + accelerationX * deltaTSimulation;
            velocityY = velocityY + accelerationY * deltaTSimulation;
            velocityZ = velocityZ + accelerationZ * deltaTSimulation;

            yCoordinate = yCoordinate + velocityY * deltaTSimulation;
            deltaX = deltaX + velocityX * deltaTSimulation;
            deltaZ = deltaZ + velocityZ * deltaTSimulation;

            time = time + deltaTSimulation;
        }

        return (deltaX, deltaZ);

    }

    /**
     * Helper functions : Purpose
     * 
     * Compute initial velocity vector: Computes the initial velocity vector of the object
     * Compute 3D Euclidean distance: Computes the Euclidean distance between 2 points in 3D space
     * Compute distance weighted score: Computes the distance weighted score using the exponential decay function
     */
    (double, double, double) computeInitialVelocityVector(double throwVelocity, double throwAngle, double facingAngle, double windSpeed)
    {
        double velocityX = throwVelocity * Math.Cos(throwAngle) * Math.Cos(facingAngle) + windSpeed;
        double velocityY = throwVelocity * Math.Sin(throwAngle);
        double velocityZ = throwVelocity * Math.Cos(throwAngle) * Math.Sin(facingAngle);

        return (velocityX, velocityY, velocityZ);
    }

    double compute3DEuclideanDistance((double x, double y, double z) pointOne, (double x, double y, double z) pointTwo)
    {
        double deltaX = pointOne.x - pointTwo.x;
        double deltaY = pointOne.y - pointTwo.y;
        double deltaZ = pointOne.z - pointTwo.z;
        return Math.Sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
    }

    double computeWeightAdjustedScore(double euclideanDistance, double lambda)
    {
        return Math.Exp(-lambda * euclideanDistance);
    }

    (double, double, double) convertVectorToTuple(Vector3 vector)
    {
        return (vector.x, vector.y, vector.z);
    }
}
