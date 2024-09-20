using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System;
using System.Collections;
using System.Collections.Generic;


public class SandboxUI : MonoBehaviour
{
    // Assign these references from the Inspector or dynamically via code
    public TMP_Dropdown chooseObjectInput;
    public TMP_InputField throwSpeedInput;
    public TMP_InputField throwAngleInput;
    public TMP_InputField faceAngleInput;
     public TMP_InputField windSpeedInput;
    public TMP_Dropdown chooseOptimum;
    public TMP_InputField landingPositionInput;
    public Button useDefaultDistributionsButton;
    public Button runSimulationButton;

    public static Boolean runEstimatorMC = false;
    public static Boolean useDefaultDistribution = false;

    public static string thrownObject;
    public static Vector3 landingPosition;
    public static double throwVelocityDistributionShape;
    public static double throwVelocityDistributionRate;
    public static double throwAngleDistributionMean;
    public static double throwAngleDistributionSD;
    public static double facingAngleDistributionMean;
    public static double facingAngleDistributionSD;
    public static double windSpeedDistributionShape;
    public static double windSpeedDistributionScale;
    public static String globalOptimumOrMeanBest;

    private SandboxController sandboxController;
    // Start is called before the first frame update
    void Start()
    {
        // Get a reference to the SandboxController script
        sandboxController = GetComponent<SandboxController>();

        if (sandboxController == null)
        {
            Debug.LogError("sandboxController not found! Make sure it's attached to the same GameObject.");
        }

        // Wire up the buttons to their respective functions
        useDefaultDistributionsButton.onClick.AddListener(useDefault);
        runSimulationButton.onClick.AddListener(onSubmit);
    }

    void useDefault(){
        useDefaultDistribution = true;
    }
       void onSubmit()
    {
        (double, double) parseDistribution(TMP_InputField input){
            string inputText = input.text;
            string[] values = inputText.Trim().Split(',');
            if(values.Length == 2){
                double x,y;
                if(double.TryParse(values[0],out x) &&
                double.TryParse(values[1], out y))
                {

                    return (x, y);
                }
            }
            return (0d,0d);
        }
        
        globalOptimumOrMeanBest = chooseOptimum.options[chooseOptimum.value].text;

        if(useDefaultDistribution == false){
            
            (throwVelocityDistributionShape, throwVelocityDistributionRate) = parseDistribution(throwSpeedInput);
            (throwAngleDistributionMean,throwAngleDistributionSD) = parseDistribution(throwAngleInput);
            (facingAngleDistributionMean,facingAngleDistributionSD) = parseDistribution(faceAngleInput);
            (windSpeedDistributionShape,windSpeedDistributionScale) = parseDistribution(windSpeedInput);

        }


        thrownObject = chooseObjectInput.options[chooseObjectInput.value].text;
        // Get the input text
        string landingPositionText = landingPositionInput.text;
        // Remove any extra whitespace and split the string by commas
        string[] values = landingPositionText.Trim().Split(',');

        // Check if we have exactly 3 components
        if (values.Length == 3)
        {
            // Try parsing the x, y, and z components into floats
            float x, y, z;
            if (float.TryParse(values[0], out x) && 
                float.TryParse(values[1], out y) && 
                float.TryParse(values[2], out z))
            {
                // Store the parsed values in the Vector3
                landingPosition = new Vector3(x, y, z);
                Debug.Log("Parsed Vector3: " + landingPosition);
            }
            else
            {
                Debug.LogError("Input could not be parsed into valid floats.");
            }
        }
        else
        {
            Debug.LogError("Input does not have exactly 3 values.");
        }

        runEstimatorMC = true;
        sandboxController.StartSimulation();

    }


}