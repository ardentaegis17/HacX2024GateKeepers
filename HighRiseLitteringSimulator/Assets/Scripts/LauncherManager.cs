using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.SceneManagement;

public class LauncherManager : MonoBehaviour
{
    public EstimatorV2 estimatorV2;
    public EstimatorMonteCarloGeneralized estimatorMonteCarloGeneralized;
    string thrownObject;
    Vector3 targetPos;

    public GameObject flowerpot;
    public GameObject bottle;
    public GameObject target;
    public static GameObject chosenObject;
    public Quaternion spawnRotation = Quaternion.identity;

    public Camera mainCamera;
    public Camera fullCamera;

    // Start is called before the first frame update
    void Start()
    {
        mainCamera.enabled = true;
        fullCamera.enabled = false;

        estimatorV2.enabled = false;
        estimatorMonteCarloGeneralized.enabled = false;

        Boolean useEstimatorV2 = QuickStartUI.runEstimatorV2;
        Boolean useEstimatorMC = SandboxUI.runEstimatorMC;

        Debug.Log("useEstimatorV2: " + useEstimatorV2);

        if(useEstimatorV2){
            Debug.Log("enabling estimatorV2");
            estimatorV2.enabled = true;
            thrownObject = QuickStartUI.thrownObject;
            targetPos = QuickStartUI.landingPosition;
        } else if (useEstimatorMC){
            Debug.Log("enabling estimatorMC");
            estimatorMonteCarloGeneralized.enabled = true;
            thrownObject = SandboxUI.thrownObject;
            targetPos = SandboxUI.landingPosition;
        }

        // instantiate object and landing point.
        
        Vector3 objectPos = targetPos + new Vector3(0,0.1f,0);

        Instantiate(target, targetPos,spawnRotation);

        switch(thrownObject){
            case "Flowerpot":
                chosenObject = Instantiate(flowerpot, objectPos, spawnRotation);;
                break;
            case "Bottle":
                chosenObject = Instantiate(bottle, objectPos, spawnRotation);
                break;
            default:
                break;
        }

        
        DontDestroyOnLoad(chosenObject);
    }

    // Update is called once per frame
    void Update()
    {
        if(Input.GetKeyDown(KeyCode.Q)){
            QuickStartUI.runEstimatorV2 = false;
            SandboxUI.runEstimatorMC = false;
            SceneManager.LoadScene("TitleScreen");
        }
        if(Input.GetKeyDown(KeyCode.Alpha1)){
            Debug.Log("Switching to Main View");
            mainCamera.enabled = true;
            fullCamera.enabled = false;
            
        }
        if(Input.GetKeyDown(KeyCode.Alpha2)){
            Debug.Log("Switching to Full View");
            mainCamera.enabled = false;
            fullCamera.enabled = true;
            
        }
        
    }
}
