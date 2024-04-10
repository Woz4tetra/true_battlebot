using RosMessageTypes.BwInterfaces;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class CollisionSensor : MonoBehaviour
{
    [SerializeField] private string collisionInfoTopic = "simulation/collision_info";
    ROSConnection ros;
    RosTopicState topicState;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        topicState = ros.GetOrCreateTopic(collisionInfoTopic, MessageRegistry.GetRosMessageName<CollisionInfoMsg>());
        if (!topicState.IsPublisher)
        {
            ros.RegisterPublisher<CollisionInfoMsg>(topicState.Topic);
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (ros == null) return;
        GameObject topLevelObject = ObjectUtils.GetTopLevelObject(other.gameObject);
        CollisionInfoMsg msg = new CollisionInfoMsg
        {
            source_object = gameObject.name,
            collision_with = topLevelObject.name
        };
        topicState.Publish(msg);
    }
}