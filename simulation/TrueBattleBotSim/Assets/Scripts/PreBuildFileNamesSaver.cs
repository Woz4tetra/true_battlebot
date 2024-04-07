using System;
using System.IO;
using System.Linq;
using UnityEngine;
using System.Collections.Generic;


#if UNITY_EDITOR
using UnityEditor.Build;
using UnityEditor;


[InitializeOnLoadAttribute]
class PrePlayModeFileNamesSaver
{
    static PrePlayModeFileNamesSaver()
    {
        EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
    }

    static void OnPlayModeStateChanged(PlayModeStateChange state)
    {
        PreBuildFileNamesSaver.SaveFilenames();
    }
}

class PreBuildFileNamesSaver : IPreprocessBuildWithReport
{
    public int callbackOrder { get { return 0; } }
    public void OnPreprocessBuild(UnityEditor.Build.Reporting.BuildReport report)
    {
        SaveFilenames();
    }

    public static void SaveFilenames()
    {
        Debug.Log("Saving file names to Resources folder");
        //The Resources folder path
        string resourcesPath = Application.dataPath + "/Resources";

        //Get file names except the ".meta" extension
        string[] fileNames = Directory.GetFiles(resourcesPath, "*", SearchOption.AllDirectories)
            .Where(x => Path.GetExtension(x) != ".meta").ToArray();
        string[] relativeFileNames = fileNames.Select(x => x.Replace(resourcesPath + "/", "")).ToArray();

        //Convert the Names to Json to make it easier to access when reading it
        FileNameInfo fileInfo = new FileNameInfo(relativeFileNames);
        string fileInfoJson = JsonUtility.ToJson(fileInfo);

        //Save the json to the Resources folder as "FileNames.txt"
        File.WriteAllText(Application.dataPath + "/Resources/FileNames.json", fileInfoJson);

        AssetDatabase.Refresh();
    }
}
#endif

[Serializable]
public class FileNameInfo
{
    public string[] fileNames;

    public FileNameInfo(string[] fileNames)
    {
        this.fileNames = fileNames;
    }

    public string[] GetFiles(string directory)
    {
        List<string> selectedFiles = new List<string>();
        for (int i = 0; i < fileNames.Length; i++)
        {
            string path = fileNames[i];
            if (path.StartsWith(directory))
            {
                string filename = Path.GetFileNameWithoutExtension(Path.GetFileName(path));
                selectedFiles.Add(filename);
            }
        }
        return selectedFiles.ToArray();
    }
}