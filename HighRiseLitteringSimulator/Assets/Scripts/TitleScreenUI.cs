using UnityEngine;
using UnityEngine.UI;

public class TitleScreenUI : MonoBehaviour
{
    // Assign these references from the Inspector or dynamically via code
    public Button startButton;
    public Button sandboxButton;
    public Button helpButton;
    public Button quitButton;

    private TitleScreenController titleScreenController;

    void Start()
    {
        // Get a reference to the TitleScreenController script
        titleScreenController = GetComponent<TitleScreenController>();

        if (titleScreenController == null)
        {
            Debug.LogError("TitleScreenController not found! Make sure it's attached to the same GameObject.");
        }

        // Wire up the buttons to their respective functions
        startButton.onClick.AddListener(() => titleScreenController.StartGame());
        sandboxButton.onClick.AddListener(() => titleScreenController.OpenSandbox());
        // helpButton.onClick.AddListener(() => titleScreenController.OpenHelp());
        quitButton.onClick.AddListener(() => titleScreenController.QuitGame());
    }
}