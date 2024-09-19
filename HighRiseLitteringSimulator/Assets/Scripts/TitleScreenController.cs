using UnityEngine;
using UnityEngine.SceneManagement;

public class TitleScreenController : MonoBehaviour
{
    // Load the game scene
    public void StartGame()
    {
        // Assuming your main game scene is named "MainGame"
        SceneManager.LoadScene("QuickStart");
    }

    // Go to the settings menu (you would need a settings scene or panel)
    public void OpenHelp()
    {
        // Assuming your settings scene is named "Settings"
        SceneManager.LoadScene("Help");
    }

    // Load Sandbox Start Screen
    public void OpenSandbox()
    {
        // Assuming your credits scene is named "Credits"
        SceneManager.LoadScene("Sandbox");
    }

    // Quit the game
    public void QuitGame()
    {
        // If running in the Unity editor, stop play mode
#if UNITY_EDITOR
        UnityEditor.EditorApplication.isPlaying = false;
#else
        // Quit the application
        Application.Quit();
#endif
    }
}