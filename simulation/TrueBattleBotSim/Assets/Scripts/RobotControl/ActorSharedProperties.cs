using UnityEngine;
public class ActorSharedProperties : MonoBehaviour
{
    [SerializeField] string sequenceProgressTopicName = "sequence_progress";

    public string GetSequenceProgressTopicName()
    {
        return sequenceProgressTopicName;
    }
}