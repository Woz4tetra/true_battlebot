using System;

[Serializable]
public class CageConfig
{
    public DimsConfig dims = new DimsConfig();
    public bool display_readout = true;
    public string cage_type = "nhrl_3lb_cage";
}