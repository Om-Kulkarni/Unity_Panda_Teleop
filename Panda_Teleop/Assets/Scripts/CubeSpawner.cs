using System.Collections.Generic;
using UnityEngine;

public class CubeSpawner : MonoBehaviour
{
    [Header("Spawning Configuration")]
    [Tooltip("The cube prefab that has all the XR Grab components.")]
    public GameObject cubePrefab;

    [Tooltip("An empty GameObject that marks the center of the 3x3 grid.")]
    public Transform spawnCenterPoint;

    [Tooltip("The distance between each cube in the grid.")]
    public float spacing = 0.5f;

    [Header("Color Palette")]
    [Tooltip("The 9 hexadecimal colors to be randomly assigned to the cubes.")]
    public List<string> hexColorCodes = new List<string>
    {
        "#FF5733", // Orange
        "#33FF57", // Green
        "#3357FF", // Blue
        "#FFFF33", // Yellow
        "#FF33FF", // Magenta
        "#33FFFF", // Cyan
        "#FFFFFF", // White
        "#808080", // Gray
        "#000000"  // Black
    };

    // Start is called before the first frame update
    void Start()
    {
        // Check if the necessary components are assigned in the Inspector.
        if (cubePrefab == null || spawnCenterPoint == null)
        {
            Debug.LogError("Cube Prefab or Spawn Center Point is not assigned. Cannot spawn cubes.");
            return;
        }

        SpawnGrid();        
    }

    /// <summary>
    /// Spawns a 3x3 grid of cubes with randomized colors.
    /// </summary>
    public void SpawnGrid()
    {
        // 1. Shuffle the list of colors to ensure random assignment without repetition.
        List<string> shuffledColors = new List<string>(hexColorCodes);
        int n = shuffledColors.Count;
        for (int i = n - 1; i > 0; i--)
        {
            int j = Random.Range(0, i + 1);
            string temp = shuffledColors[i];
            shuffledColors[i] = shuffledColors[j];
            shuffledColors[j] = temp;
        }

        int colorIndex = 0;

        // 2. Create the 3x3 grid using nested loops.
        for (int x = 0; x < 3; x++)
        {
            for (int z = 0; z < 3; z++)
            {
                // 3. Calculate the position for each cube.
                // The (x-1) and (z-1) logic centers the grid, making coordinates go from -1 to 1.
                Vector3 spawnPosition = spawnCenterPoint.position + new Vector3((x - 1) * spacing, 0, (z - 1) * spacing);

                // 4. Instantiate a new cube from the prefab.
                GameObject newCube = Instantiate(cubePrefab, spawnPosition, Quaternion.identity);
                newCube.transform.SetParent(this.transform); // Parent to the spawner for a clean hierarchy.

                // 5. Get the new cube's renderer and apply the color.
                Renderer cubeRenderer = newCube.GetComponent<Renderer>();
                if (cubeRenderer != null)
                {
                    // This creates a new material instance so we don't change the prefab's material.
                    if (ColorUtility.TryParseHtmlString(shuffledColors[colorIndex], out Color newColor))
                    {
                        cubeRenderer.material.color = newColor;
                    }
                }
                colorIndex++;
            }
        }
    }
}
