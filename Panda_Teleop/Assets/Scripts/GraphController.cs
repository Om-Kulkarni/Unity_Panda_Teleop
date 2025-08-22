using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class GraphController : MonoBehaviour
{
    // UI element to display the graph
    public RawImage graphImage;
    // Color of the graph line
    public Color graphColor = Color.blue;
    // The speed at which the graph moves in pixels/sec
    public float graphSpeed = 50f;
    // The height of the graph
    public float graphHeight = 100f;
    // The amount of noise in the graph
    public float noiseAmount = 5f;
    // The chance of a spike appearing
    [Range(0, 1)]
    public float spikeChance = 0.01f;
    // The minimum height of a spike
    public float minSpikeHeight = 20f;
    // The maximum height of a spike
    public float maxSpikeHeight = 50f;

    //  Controls the vertical position of the baseline
    [Tooltip("Vertical position of the baseline, from 0 (bottom) to 1 (top).")]
    [Range(0f, 1f)]
    public float baselinePosition = 0.15f; // Default to 15% from the bottom

    private Texture2D graphTexture;
    private List<float> graphData;
    private int textureWidth;
    private int textureHeight;

    // --- Variables for Tracking Spikes ---
    [Header("Spike Tracking")]
    public int spikesOccurred = 0;
    public int spikesSaved = 0;

    // Timer to control update frequency
    private float timeSinceLastUpdate = 0f;

    // Reference to the database manager
    [Tooltip("Reference to the Database Manager.")]
    public DatabaseManager databaseManager;

    // Start is called before the first frame update
    void Start()
    {
        // Get the dimensions of the RawImage
        textureWidth = (int)graphImage.rectTransform.rect.width;
        textureHeight = (int)graphImage.rectTransform.rect.height;

        // Create a new texture to draw the graph on
        graphTexture = new Texture2D(textureWidth, textureHeight);
        graphImage.texture = graphTexture;

        // Initialize the graph data
        graphData = new List<float>();
        float InitialY = textureHeight * baselinePosition; // Calculate the initial Y position based on the baseline
        for (int i = 0; i < textureWidth; i++)
        {
            graphData.Add(InitialY);
        }

        // Clear the texture
        ClearTexture();
    }

    // Update is called once per frame
    void Update()
    {
        // Add to the timer
        timeSinceLastUpdate += Time.deltaTime;

        // Time required for each update step
        float timePerUpdate = 1f / graphSpeed;

        if (graphSpeed <= 0)
        {
            return; // Skip this update if the speed is zero
        }

        // Use a while loop to update the graph data
        while (timeSinceLastUpdate >= timePerUpdate)
        {
            timeSinceLastUpdate -= timePerUpdate;

            // Move the graph data
            for (int i = 0; i < graphData.Count - 1; i++)
            {
                graphData[i] = graphData[i + 1];
            }

            // Calculate the base noise. 
            // We also use graphHeight as a general multiplier for the scale.
            float noiseValue = Random.Range(-noiseAmount, noiseAmount) * (graphHeight / 100f);

            // 2. Check if a spike should be generated
            if (Random.value < spikeChance)
            {
                // Calculate a random spike height
                float spikeValue = Random.Range(minSpikeHeight, maxSpikeHeight) * (graphHeight / 30f);
                noiseValue += spikeValue;
                spikesOccurred++;
            }

            // 3. Set the final data point, using the texture's midpoint as the baseline
            float newDataPoint = baselinePosition * textureHeight + noiseValue;

            graphData[graphData.Count - 1] = newDataPoint;

            // Redraw the graph
            DrawGraph();
        }
    }

    // This function is called when the object is destroyed, including when the game stops.
    private void OnDestroy()
    {
    }

    // Clears the texture to a transparent color
    void ClearTexture()
    {
        Color32[] clearColorArray = new Color32[textureWidth * textureHeight];
        for (int i = 0; i < clearColorArray.Length; i++)
        {
            clearColorArray[i] = new Color32(0, 0, 0, 0);
        }
        graphTexture.SetPixels32(clearColorArray);
        graphTexture.Apply();
    }

    // Draws the graph on the texture
    void DrawGraph()
    {
        ClearTexture();

        for (int i = 0; i < graphData.Count - 1; i++)
        {
            DrawLine(new Vector2(i, graphData[i]), new Vector2(i + 1, graphData[i + 1]), graphColor);
        }

        graphTexture.Apply();
    }

    // Draws a line on the texture
    void DrawLine(Vector2 start, Vector2 end, Color color)
    {
        int x0 = (int)start.x;
        int y0 = (int)start.y;
        int x1 = (int)end.x;
        int y1 = (int)end.y;

        int dx = Mathf.Abs(x1 - x0);
        int dy = Mathf.Abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            if (x0 >= 0 && x0 < textureWidth && y0 >= 0 && y0 < textureHeight)
            {
                graphTexture.SetPixel(x0, y0, color);
            }

            if ((x0 == x1) && (y0 == y1)) break;
            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }
    }
    
    /// <summary>
    /// This function should be called by the 'Save Spike' button's OnClick event.
    /// </summary>
    public void OnSaveSpikeClicked()
    {
        spikesSaved++;
        // You can add other logic here, like checking if a spike was actually on screen
        Debug.Log("Spike Saved! Total Saved: " + spikesSaved);
    }    
}
