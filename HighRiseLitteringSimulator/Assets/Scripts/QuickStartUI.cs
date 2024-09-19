using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System;
using System.Collections;
using System.Collections.Generic;

public class QuickStartUI : MonoBehaviour
{
    // Assign these references from the Inspector or dynamically via code
    public TMP_Dropdown chooseObjectInput;
    public TMP_Dropdown throwSpeedInput;
    public TMP_Dropdown throwAngleInput;
    public TMP_Dropdown faceAngleInput;
     public TMP_InputField windSpeedInput;
    public TMP_Dropdown windDirectionInput;
    public TMP_InputField landingPositionInput;
    public Button runSimulationButton;

    public static Boolean runEstimatorV2 = false;

    public static string thrownObject;
    public static int throwSpeed;
    public static float throwAngle;
    public static float faceAngle;
    public static int windSpeed;
    public static int windDirection;
    public static Vector3 landingPosition;


    private QuickStartController quickStartController;

    void Start()
    {
        // Get a reference to the QuickStartController script
       quickStartController = GetComponent<QuickStartController>();

        if (quickStartController == null)
        {
            Debug.LogError("QuickStartController not found! Make sure it's attached to the same GameObject.");
        }

        // Wire up the buttons to their respective functions
        runSimulationButton.onClick.AddListener(onSubmit);


    }
    void onSubmit()
    {
        thrownObject = chooseObjectInput.options[chooseObjectInput.value].text;

        if (throwSpeedInput.options[throwSpeedInput.value].text == "Soft (2 m/s)"){
            throwSpeed = 2;
        }else if (throwSpeedInput.options[throwSpeedInput.value].text == "Moderate (8 m/s)"){
            throwSpeed = 8;
        } else if (throwSpeedInput.options[throwSpeedInput.value].text == "Hard (15 m/s)"){
            throwSpeed = 15;
        }
        else{
            throwSpeed = 0;
        }

        string throwAngleChoice = throwAngleInput.options[throwAngleInput.value].text;
        switch(throwAngleChoice){
            case "Down (-45)":
                throwAngle = -45f;
                break;
            case "Slightly Down (-15)":
                throwAngle = -15f;
                break;
            case "Straight (0)":
                throwAngle = 0f;
                break;
            case "Slightly Up (15)":
                throwAngle = 15f;
                break;
            case "Up (45)":
                throwAngle = 45f;
                break;
            default:
                throwAngle = 0f;
                break;
        }

        string faceAngleChoice = faceAngleInput.options[faceAngleInput.value].text;
        switch(faceAngleChoice){
            case "Facing Left (45)":
                faceAngle = 45f;
                break;
            case "Facing Forward (90)":
                faceAngle = 90f;
                break;
            case "Facing Right (135)":
                faceAngle = 135f;
                break;
            default:
                faceAngle = 90f;
                break;
        }


        windSpeed = int.Parse(windSpeedInput.text);
        if (windDirectionInput.options[windDirectionInput.value].text == "Left"){
            windDirection = 0;
        } else{
            windDirection = 1;
        }

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
        Debug.Log("thrownObject: " + thrownObject + " throwSpeed: " + throwSpeed + " windSpeed: " + windSpeed + " windDirection: " + windDirection + " landingPosition: " + landingPosition);
        runEstimatorV2 = true;
        quickStartController.StartSimulation();
    }


}