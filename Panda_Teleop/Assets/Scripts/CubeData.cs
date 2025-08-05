using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Represents the properties of a single cube configuration.
/// The [System.Serializable] attribute allows this class to be converted to and from JSON.
/// </summary>
[System.Serializable]
public class CubeData
{
    public Color cubeColor;
    public float cubeSize;
    public string cubeTexture;
    public string cubeQuality;
}

/// <summary>
/// A wrapper class that holds a list of CubeData objects.
/// This is a common pattern needed to correctly serialize a list into a JSON array.
/// </summary>
[System.Serializable]
public class CubeDataList
{
    public List<CubeData> allCubes;

    public CubeDataList()
    {
        allCubes = new List<CubeData>();
    }
}