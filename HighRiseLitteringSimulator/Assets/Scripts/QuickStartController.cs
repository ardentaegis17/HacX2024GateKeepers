using UnityEngine;
using UnityEngine.SceneManagement;

public class QuickStartController : MonoBehaviour
{
    // Begin the Simulation
    public void StartSimulation()
    {
        // Assuming your main game scene is named "MainGame"
        SceneManager.LoadScene("Simulation");
    }
}
