using System;

[Serializable]
public class ActorConfig
{
    public string name = "mini_bot";
    public string model = "Mini bot";

    /**
     * Actor objective.
     *  keyboard: Keyboard control.
     *  follow: Follow a sequence of waypoints.
     *  target: Follow another actor.
     *  idle: Do nothing.
     *  auto: Take commands from ROS.
     *  teleport: Teleport to a sequence of waypoints.
     *  relative_to: Set parent object to another actor.
     */
    public string objective = "";
}