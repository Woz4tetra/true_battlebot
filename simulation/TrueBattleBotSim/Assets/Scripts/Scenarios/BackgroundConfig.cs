using System;

[Serializable]
public class BackgroundConfig
{
    public string name = "Garage Scene";
    public string sky_image = "";

    public override bool Equals(object obj)
    {
        if (obj is BackgroundConfig)
        {
            BackgroundConfig other = (BackgroundConfig)obj;
            return name == other.name && sky_image == other.sky_image;
        }
        return base.Equals(obj);
    }

    public override int GetHashCode()
    {
        return name.GetHashCode() ^ sky_image.GetHashCode();
    }
}