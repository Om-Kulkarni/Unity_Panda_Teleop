using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class GraphController : MonoBehaviour
{
    // UI element to display the graph
    public RawImage graphImage;
    // Color of the graph line
    public Color graphColor = Color.blue;
    // The speed at which the graph moves
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

    private Texture2D graphTexture;
    private List<float> graphData;
    private int textureWidth;
    private int textureHeight;

    // --- Variables for Tracking Spikes ---
    [Header("Spike Tracking")]
    public int spikesOccurred = 0;
    public int spikesSaved = 0;

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
        for (int i = 0; i < textureWidth; i++)
        {
            graphData.Add(textureHeight / 2f);
        }

        // Clear the texture
        ClearTexture();
    }

    // Update is called once per frame
    void Update()
    {
        // Move the graph data
        for (int i = 0; i < graphData.Count - 1; i++)
        {
            graphData[i] = graphData[i + 1];
        }

        // Add a new data point with noise
        float newDataPoint = textureHeight / 2f + Random.Range(-noiseAmount, noiseAmount);

        // Check if a spike should be generated
        if (Random.value < spikeChance)
        {
            newDataPoint += Random.Range(minSpikeHeight, maxSpikeHeight);
            spikesOccurred++;
        }
        graphData[graphData.Count - 1] = newDataPoint;

        // Redraw the graph
        DrawGraph();
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
